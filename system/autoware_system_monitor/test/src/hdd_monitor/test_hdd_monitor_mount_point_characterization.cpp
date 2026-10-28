// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Characterization test for how HddMonitor resolves the configured mount point
// (`disks.diskN.name`) to a block device. Pure black box: the node is loaded through its
// component plugin, configured through parameters, and observed only through /diagnostics.
// No production header is included and no private member is reached into.
//
// HOW TO READ A RUN. Each case points the monitor at a mount point discovered from
// /proc/self/mountinfo at run time, so what actually gets checked depends on what this machine
// has mounted. A case with nothing to run against SKIPS, and a skip means "not checked here" --
// never "covered", and never "broken". What each case needs:
//
//   most cases          any device-backed partition other than "/"
//   BindMountedDisk_*   a bind mount, or a btrfs subvolume
//   DeviceMapperDisk_*  a mounted LUKS (or LVM) volume, so in practice a real ECU
//
// So a green run on a workstation says nothing at all about the LUKS path -- that case will
// have skipped. Read the skip list before concluding anything about it. For the same reason
// the partition suffix strip is only ever checked for the disk type this host happens to have,
// NVMe or SATA, never both.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace
{
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;

constexpr char package_name[] = "autoware_system_monitor";
constexpr char plugin_name[] = "HddMonitor";

// The diagnostic task name the monitor registers for the connection check.
constexpr char connection_task_name[] = "HDD Connection";

// Values of HddMonitor::connection_dict_. A status carrying one of these is a real update;
// anything else (e.g. the updater's start-up placeholder) is still settling.
constexpr char connected_message[] = "OK";
constexpr char not_connected_message[] = "not connected";

constexpr std::chrono::seconds diagnostics_timeout{15};

// Loads the component library once for the whole test binary; nullptr if the plugin is not
// registered. The manager and factory are leaked on purpose: class_loader unloads the library
// in its destructor, which runs after rclcpp::shutdown() and aborts the process.
std::shared_ptr<rclcpp_components::NodeFactory> get_component_factory()
{
  static auto * factory = [] {
    auto * cached = new std::shared_ptr<rclcpp_components::NodeFactory>();
    auto * manager = new rclcpp_components::ComponentManager();
    for (const auto & resource : manager->get_component_resources(package_name)) {
      if (resource.first == plugin_name) {
        *cached = manager->create_component_factory(resource);
        break;
      }
    }
    return cached;
  }();
  return *factory;
}

// GoogleTest here shows nothing of GTEST_SKIP()'s message -- not on the console, not in the
// XML it writes -- so a skipped case would look like it had simply vanished. Say why first.
void announce_skip(const std::string & reason)
{
  std::cout << "[   REASON ] " << reason << std::endl;
}

// A mount point taken from the running system, together with the device backing it.
struct MountedDevice
{
  std::string mount_point;
  std::string device;
  // True when the filesystem is mounted from a subdirectory of itself (a bind mount or a
  // subvolume) rather than from its root.
  bool is_bind{false};
};

bool is_whole_disk_capable(const std::string & device)
{
  return device.rfind("/dev/sd", 0) == 0 || device.rfind("/dev/nvme", 0) == 0 ||
         device.rfind("/dev/mmcblk", 0) == 0;
}

// Reads this machine's device-backed mount points from /proc/self/mountinfo. mountinfo rather
// than /proc/mounts because only mountinfo carries the "root" field that distinguishes a bind
// mount from a plain one, and the two resolve differently.
//
// Lines containing octal-escaped characters are skipped so that the test never has to unescape
// anything, which would mean reimplementing production logic inside the test.
std::vector<MountedDevice> read_mounted_devices()
{
  std::vector<MountedDevice> devices;

  std::ifstream mountinfo("/proc/self/mountinfo");
  if (!mountinfo.is_open()) {
    return devices;
  }

  std::string line;
  while (std::getline(mountinfo, line)) {
    if (line.find('\\') != std::string::npos) {
      continue;
    }

    std::vector<std::string> fields;
    std::istringstream iss(line);
    std::string token;
    while (iss >> token) {
      fields.push_back(token);
    }

    // id parent major:minor root mount_point options [optional fields] - fstype source superopts
    const auto separator = std::find(fields.begin(), fields.end(), "-");
    if (fields.size() < 6 || separator == fields.end()) {
      continue;
    }
    if (std::distance(separator, fields.end()) < 3) {
      continue;
    }

    const std::string & root = fields[3];
    const std::string & mount_point = fields[4];
    const std::string & source = *(separator + 2);

    if (source.rfind("/dev/", 0) != 0) {
      continue;  // needs a real device node behind it
    }

    std::error_code ec;
    if (!std::filesystem::exists(source, ec) || !std::filesystem::is_directory(mount_point, ec)) {
      continue;
    }

    devices.push_back(MountedDevice{mount_point, source, root != "/"});
  }

  return devices;
}

// A plainly mounted partition to point the monitor at. A real S.M.A.R.T.-capable device is
// preferred so the scenario matches an actual ECU, but any device node will do.
std::optional<MountedDevice> find_plain_mount_point()
{
  std::optional<MountedDevice> fallback;
  for (const auto & device : read_mounted_devices()) {
    // "/" is skipped here only because it has no distinct "<mount point>/" spelling to test.
    if (device.is_bind || device.mount_point == "/") {
      continue;
    }
    if (is_whole_disk_capable(device.device)) {
      return device;
    }
    if (!fallback) {
      fallback = device;
    }
  }
  return fallback;
}

// A device-mapper backed mount: a LUKS volume, or plain LVM, which resolves through the very
// same sysfs "slaves" lookup. "/" is included -- an encrypted root is the commonest layout.
std::optional<MountedDevice> find_device_mapper_mount_point()
{
  for (const auto & device : read_mounted_devices()) {
    if (device.device.rfind("/dev/mapper/", 0) == 0 || device.device.rfind("/dev/dm-", 0) == 0) {
      return device;
    }
  }
  return std::nullopt;
}

std::optional<MountedDevice> find_bind_mount_point()
{
  for (const auto & device : read_mounted_devices()) {
    if (device.is_bind) {
      return device;
    }
  }
  return std::nullopt;
}

}  // namespace

class HddMonitorMountPointTest : public ::testing::Test
{
protected:
  // What the "HDD Connection" diagnostic reported for the single configured disk.
  struct ConnectionDiag
  {
    int level{DiagStatus::ERROR};
    std::string message;
    std::string device_name;  // value of "HDD 0: name"
    std::string mount_point;  // value of "HDD 0: mount point"
  };

  void SetUp() override
  {
    ASSERT_NE(get_component_factory(), nullptr)
      << "component " << plugin_name << " is not registered in package " << package_name;

    mounted_ = find_plain_mount_point();
    if (!mounted_) {
      announce_skip("no plainly mounted, device-backed partition other than \"/\" here");
      GTEST_SKIP();
    }

    temp_dir_ = std::filesystem::temp_directory_path() /
                ("hdd_monitor_characterization_" + std::to_string(::getpid()));
    std::filesystem::create_directories(temp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(temp_dir_, ec);
  }

  // Brings the monitor up with a single disk configured at `mount_point` and returns the first
  // settled "HDD Connection" diagnostic it publishes.
  ConnectionDiag connection_diag_for(const std::string & mount_point)
  {
    auto test_node = std::make_shared<rclcpp::Node>("characterization_driver");
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(test_node);

    std::optional<ConnectionDiag> received;
    auto subscription = test_node->create_subscription<DiagnosticArray>(
      "/diagnostics", rclcpp::QoS{10}, [&received](const DiagnosticArray::ConstSharedPtr msg) {
        for (const auto & status : msg->status) {
          if (!ends_with(status.name, connection_task_name)) {
            continue;
          }
          if (status.message != connected_message && status.message != not_connected_message) {
            continue;  // updater start-up placeholder, not a real result yet
          }
          ConnectionDiag diag;
          diag.level = status.level;
          diag.message = status.message;
          for (const auto & value : status.values) {
            if (value.key == "HDD 0: name") {
              diag.device_name = value.value;
            } else if (value.key == "HDD 0: mount point") {
              diag.mount_point = value.value;
            }
          }
          received = diag;
          return;
        }
      });

    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("num_disks", 1), rclcpp::Parameter("disks.disk0.name", mount_point),
       // Point the reader socket at a path that cannot exist, so the test never talks to a
       // privileged hdd_reader daemon -- in particular it can never trigger a real unmount.
       rclcpp::Parameter(
         "hdd_reader_socket_path", (temp_dir_ / "absent_hdd_reader.sock").string())});

    auto node_wrapper = get_component_factory()->create_node_instance(options);
    executor->add_node(node_wrapper.get_node_base_interface());

    const auto deadline = std::chrono::steady_clock::now() + diagnostics_timeout;
    while (!received && std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
      executor->spin_some(std::chrono::milliseconds(20));
    }

    executor->remove_node(node_wrapper.get_node_base_interface());
    executor->remove_node(test_node);

    EXPECT_TRUE(received.has_value())
      << "no settled \"" << connection_task_name << "\" diagnostic within "
      << diagnostics_timeout.count() << "s for mount point [" << mount_point << "]";
    return received.value_or(ConnectionDiag{});
  }

  static bool ends_with(const std::string & text, const std::string & suffix)
  {
    return text.size() >= suffix.size() &&
           text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
  }

  std::optional<MountedDevice> mounted_;
  std::filesystem::path temp_dir_;
};

// Baseline: the mount point exactly as /proc/self/mounts spells it.
TEST_F(HddMonitorMountPointTest, CanonicalMountPoint_IsConnected)
{
  // Arrange
  const auto mount_point = mounted_->mount_point;

  // Act
  const auto diag = connection_diag_for(mount_point);

  // Assert
  EXPECT_EQ(diag.level, DiagStatus::OK) << "mount point [" << mount_point << "]";
  EXPECT_EQ(diag.message, connected_message);
  EXPECT_EQ(diag.mount_point, mount_point);
}

// A trailing slash is an ordinary way to write a directory in YAML. `findmnt` canonicalized
// the target before looking it up, so this used to resolve exactly like the bare spelling.
TEST_F(HddMonitorMountPointTest, TrailingSlashMountPoint_IsConnected)
{
  // Arrange
  const auto mount_point = mounted_->mount_point + "/";

  // Act
  const auto diag = connection_diag_for(mount_point);

  // Assert
  EXPECT_EQ(diag.level, DiagStatus::OK)
    << "mount point [" << mount_point << "] must resolve like [" << mounted_->mount_point << "]";
  EXPECT_EQ(diag.message, connected_message);
}

// Likewise for redundant separators.
TEST_F(HddMonitorMountPointTest, DuplicatedSlashMountPoint_IsConnected)
{
  // Arrange
  std::string mount_point = mounted_->mount_point;
  const auto separator = mount_point.find('/', 1);
  if (separator == std::string::npos) {
    announce_skip("mount point [" + mount_point + "] has no interior separator to duplicate");
    GTEST_SKIP();
  }
  mount_point.insert(separator, "/");

  // Act
  const auto diag = connection_diag_for(mount_point);

  // Assert
  EXPECT_EQ(diag.level, DiagStatus::OK)
    << "mount point [" << mount_point << "] must resolve like [" << mounted_->mount_point << "]";
  EXPECT_EQ(diag.message, connected_message);
}

// A mount point reached through a symbolic link. `findmnt` resolved symlinks in the target.
TEST_F(HddMonitorMountPointTest, SymlinkedMountPoint_IsConnected)
{
  // Arrange
  const auto mount_point = temp_dir_ / "mount_point_link";
  std::error_code ec;
  std::filesystem::create_directory_symlink(mounted_->mount_point, mount_point, ec);
  ASSERT_FALSE(ec) << "could not create symlink " << mount_point << ": " << ec.message();

  // Act
  const auto diag = connection_diag_for(mount_point.string());

  // Assert
  EXPECT_EQ(diag.level, DiagStatus::OK)
    << "symlink [" << mount_point << "] -> [" << mounted_->mount_point << "] must resolve alike";
  EXPECT_EQ(diag.message, connected_message);
}

// Negative control: keeps the assertions above from passing vacuously. A path that is not a
// mount point has always been reported as not connected.
TEST_F(HddMonitorMountPointTest, NonMountPointPath_IsNotConnected)
{
  // Arrange
  const auto mount_point = (temp_dir_ / "not_a_mount_point").string();

  // Act
  const auto diag = connection_diag_for(mount_point);

  // Assert
  EXPECT_EQ(diag.level, DiagStatus::WARN);
  EXPECT_EQ(diag.message, not_connected_message);
}

// The resolved disk device is what hdd_reader is asked about and what /sys/block/<dev>/stat is
// read from, so an empty one silently disables S.M.A.R.T. and transfer statistics.
TEST_F(HddMonitorMountPointTest, ConnectedDisk_ReportsWholeDiskDevice)
{
  // Arrange
  if (!is_whole_disk_capable(mounted_->device)) {
    // Quirk, recorded deliberately: for loop/device-mapper/md sources the pre-existing code
    // left the disk device empty, so there is no non-empty value to characterize here.
    announce_skip(
      "backing device [" + mounted_->device +
      "] is not an ATA/NVMe/eMMC disk, so the partition "
      "suffix strip is NOT covered by this run");
    GTEST_SKIP();
  }
  const auto mount_point = mounted_->mount_point;

  // Act
  const auto diag = connection_diag_for(mount_point);

  // Assert
  ASSERT_EQ(diag.level, DiagStatus::OK);
  EXPECT_FALSE(diag.device_name.empty()) << "no disk device resolved for [" << mount_point << "]";
  EXPECT_EQ(diag.device_name.rfind("/dev/", 0), 0u)
    << "disk device [" << diag.device_name << "] is not a /dev path";
  // The partition index is stripped: /dev/nvme0n1p3 -> /dev/nvme0n1, /dev/sda2 -> /dev/sda.
  EXPECT_NE(diag.device_name, mounted_->device)
    << "disk device [" << diag.device_name << "] still carries the partition index";
}

// `findmnt -o SOURCE` rendered a bind mount (and a btrfs subvolume) as "/dev/sda1[/sub/path]",
// which is not a path: the existence check failed, the disk read as disconnected, and the
// monitor then asked hdd_reader to unmount a healthy one. Records a fix, not a regression.
TEST_F(HddMonitorMountPointTest, BindMountedDisk_IsConnected)
{
  // Arrange
  const auto bind_mount = find_bind_mount_point();
  if (!bind_mount) {
    announce_skip(
      "no device-backed bind mount here, so the bind-mount path is NOT covered by "
      "this run");
    GTEST_SKIP();
  }

  // Act
  const auto diag = connection_diag_for(bind_mount->mount_point);

  // Assert
  EXPECT_EQ(diag.level, DiagStatus::OK)
    << "bind mount [" << bind_mount->mount_point << "] on [" << bind_mount->device << "]";
  EXPECT_EQ(diag.message, connected_message);
}

// A LUKS volume mounts from /dev/mapper/<name>, which has no S.M.A.R.T. data and no
// /sys/block/<dev>/stat of its own, so the backing disk has to be reported instead. Records a
// fix, not a regression: before the change that name came back empty. Skipped, CI included,
// wherever no device-mapper volume is mounted.
TEST_F(HddMonitorMountPointTest, DeviceMapperDisk_ResolvesToBackingDisk)
{
  // Arrange
  const auto mapped = find_device_mapper_mount_point();
  if (!mapped) {
    announce_skip(
      "no device-mapper volume mounted here, so the LUKS resolution path is NOT "
      "covered by this run");
    GTEST_SKIP();
  }

  // Act
  const auto diag = connection_diag_for(mapped->mount_point);

  // Assert
  EXPECT_EQ(diag.level, DiagStatus::OK)
    << "mapped volume [" << mapped->mount_point << "] on [" << mapped->device << "]";
  EXPECT_EQ(diag.message, connected_message);
  EXPECT_FALSE(diag.device_name.empty())
    << "no backing disk resolved for [" << mapped->device << "]";
  EXPECT_EQ(diag.device_name.rfind("/dev/mapper/", 0), std::string::npos)
    << "reported [" << diag.device_name << "], which is still the mapper name";
  // Resolution is one level deep, so LUKS stacked on LVM still stops at a /dev/dm-* device.
  EXPECT_EQ(diag.device_name.rfind("/dev/dm-", 0), std::string::npos)
    << "reported [" << diag.device_name << "], which is still a device-mapper device";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
