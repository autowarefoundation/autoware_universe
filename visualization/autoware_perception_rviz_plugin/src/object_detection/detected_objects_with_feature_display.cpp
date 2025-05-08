// Copyright 2021 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "autoware_perception_rviz_plugin/object_detection/detected_objects_with_feature_display.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <qcolor.h>
#include <qobject.h>

#include <rclcpp/duration.hpp>
#include <QObject>
#include <memory>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
  inline std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f)
  {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }

  std_msgs::msg::ColorRGBA generateDistinctColor(size_t idx)
  {
    constexpr float HUE_SPACING = 47.0f; // A prime number often works well for spacing
    constexpr float MAX_HUE = 360.0f;
    constexpr float SATURATION = 0.9f; // Slightly desaturated can be more pleasant
    constexpr float VALUE = 0.95f;    // Not full brightness to avoid harsh colors

    float hue = std::fmod(static_cast<float>(idx * HUE_SPACING), MAX_HUE);
    
    float c = VALUE * SATURATION;
    float x = c * (1.0f - std::fabs(std::fmod(hue / 60.0f, 2.0f) - 1.0f));
    float m = VALUE - c;

    float r_prime = 0, g_prime = 0, b_prime = 0;

    if (hue < 60.0f)      { r_prime = c; g_prime = x; }
    else if (hue < 120.0f){ r_prime = x; g_prime = c; }
    else if (hue < 180.0f){ g_prime = c; b_prime = x; } // Fixed: was r_prime = 0, g_prime = c, b_prime = x;
    else if (hue < 240.0f){ g_prime = x; b_prime = c; } // Fixed: was r_prime = 0, g_prime = x, b_prime = c;
    else if (hue < 300.0f){ r_prime = x; b_prime = c; }
    else                  { r_prime = c; b_prime = x; } // Fixed: was r_prime = c, g_prime = 0, b_prime = x;
  
    return makeColor(r_prime + m, g_prime + m, b_prime + m);
  }

  std_msgs::msg::ColorRGBA colormapJet(float value_normalized)
  {
    value_normalized = std::clamp(value_normalized, 0.0f, 1.0f);
    float r = std::clamp(1.5f - std::abs(4.0f * value_normalized - 3.0f), 0.0f, 1.0f);
    float g = std::clamp(1.5f - std::abs(4.0f * value_normalized - 2.0f), 0.0f, 1.0f);
    float b = std::clamp(1.5f - std::abs(4.0f * value_normalized - 1.0f), 0.0f, 1.0f);  
    return makeColor(r,g,b);
  }
  
  std_msgs::msg::ColorRGBA colormapViridis(float v)
  {
    v = std::clamp(v, 0.0f, 1.0f);
    // User's linear interpolation for Viridis approximation
    float r = std::clamp(0.267f + v * (0.993f - 0.267f), 0.0f, 1.0f);
    float g = std::clamp(0.004f + v * (0.906f - 0.004f), 0.0f, 1.0f);
    float b = std::clamp(0.329f + v * (0.933f - 0.329f), 0.0f, 1.0f);
    return makeColor(r,g,b);
  }
  
  std_msgs::msg::ColorRGBA colormapHSV(float v)
  {
    v = std::clamp(v, 0.0f, 1.0f);
    float h = v * 360.0f;  // hue in degrees
    float s = 1.0f, l = 0.5f;
  
    float c = (1.0f - std::fabs(2.0f * l - 1.0f)) * s;
    float x = c * (1.0f - std::fabs(std::fmod(h / 60.0f, 2) - 1.0f));
    float m = l - c / 2.0f;
  
    float r_prime = 0, g_prime = 0, b_prime = 0;
    if (h < 60.0f)      { r_prime = c; g_prime = x; }
    else if (h < 120.0f){ r_prime = x; g_prime = c; }
    else if (h < 180.0f){ g_prime = c; b_prime = x; }
    else if (h < 240.0f){ g_prime = x; b_prime = c; }
    else if (h < 300.0f){ r_prime = x; b_prime = c; }
    else                { r_prime = c; b_prime = x; }
  
    return makeColor(r_prime + m, g_prime + m, b_prime + m);
  }

  std_msgs::msg::ColorRGBA colormapRed(float v)
  {
    return makeColor(std::clamp(v, 0.0f, 1.0f), 0.0f, 0.0f);
  }

  std_msgs::msg::ColorRGBA colormapGray(float value)
  {
    value = std::clamp(value, 0.0f, 1.0f);
    return makeColor(value,value,value);
  }

  std_msgs::msg::ColorRGBA colormapTurbo(float x)
  {
    x = std::clamp(x, 0.0f, 1.0f);
  
    constexpr float turbo[256][3] =
    {
        {0.18995f, 0.07176f, 0.23217f},
        {0.19483f, 0.08339f, 0.26149f},
        {0.19956f, 0.09498f, 0.29024f},
        {0.20415f, 0.10652f, 0.31844f},
        {0.20860f, 0.11802f, 0.34607f},
        {0.21291f, 0.12947f, 0.37314f},
        {0.21708f, 0.14087f, 0.39964f},
        {0.22111f, 0.15223f, 0.42558f},
        {0.22500f, 0.16354f, 0.45096f},
        {0.22875f, 0.17481f, 0.47578f},
        {0.23236f, 0.18603f, 0.50004f},
        {0.23582f, 0.19720f, 0.52373f},
        {0.23915f, 0.20833f, 0.54686f},
        {0.24234f, 0.21941f, 0.56942f},
        {0.24539f, 0.23044f, 0.59142f},
        {0.24830f, 0.24143f, 0.61286f},
        {0.25107f, 0.25237f, 0.63374f},
        {0.25369f, 0.26327f, 0.65406f},
        {0.25618f, 0.27412f, 0.67381f},
        {0.25853f, 0.28492f, 0.69300f},
        {0.26074f, 0.29568f, 0.71162f},
        {0.26280f, 0.30639f, 0.72968f},
        {0.26473f, 0.31706f, 0.74718f},
        {0.26652f, 0.32768f, 0.76412f},
        {0.26816f, 0.33825f, 0.78050f},
        {0.26967f, 0.34878f, 0.79631f},
        {0.27103f, 0.35926f, 0.81156f},
        {0.27226f, 0.36970f, 0.82624f},
        {0.27334f, 0.38008f, 0.84037f},
        {0.27429f, 0.39043f, 0.85393f},
        {0.27509f, 0.40072f, 0.86692f},
        {0.27576f, 0.41097f, 0.87936f},
        {0.27628f, 0.42118f, 0.89123f},
        {0.27667f, 0.43134f, 0.90254f},
        {0.27691f, 0.44145f, 0.91328f},
        {0.27701f, 0.45152f, 0.92347f},
        {0.27698f, 0.46153f, 0.93309f},
        {0.27680f, 0.47151f, 0.94214f},
        {0.27648f, 0.48144f, 0.95064f},
        {0.27603f, 0.49132f, 0.95857f},
        {0.27543f, 0.50115f, 0.96594f},
        {0.27469f, 0.51094f, 0.97275f},
        {0.27381f, 0.52069f, 0.97899f},
        {0.27273f, 0.53040f, 0.98461f},
        {0.27106f, 0.54015f, 0.98930f},
        {0.26878f, 0.54995f, 0.99303f},
        {0.26592f, 0.55979f, 0.99583f},
        {0.26252f, 0.56967f, 0.99773f},
        {0.25862f, 0.57958f, 0.99876f},
        {0.25425f, 0.58950f, 0.99896f},
        {0.24946f, 0.59943f, 0.99835f},
        {0.24427f, 0.60937f, 0.99697f},
        {0.23874f, 0.61931f, 0.99485f},
        {0.23288f, 0.62923f, 0.99202f},
        {0.22676f, 0.63913f, 0.98851f},
        {0.22039f, 0.64901f, 0.98436f},
        {0.21382f, 0.65886f, 0.97959f},
        {0.20708f, 0.66866f, 0.97423f},
        {0.20021f, 0.67842f, 0.96833f},
        {0.19326f, 0.68812f, 0.96190f},
        {0.18625f, 0.69775f, 0.95498f},
        {0.17923f, 0.70732f, 0.94761f},
        {0.17223f, 0.71680f, 0.93981f},
        {0.16529f, 0.72620f, 0.93161f},
        {0.15844f, 0.73551f, 0.92305f},
        {0.15173f, 0.74472f, 0.91416f},
        {0.14519f, 0.75381f, 0.90496f},
        {0.13886f, 0.76279f, 0.89550f},
        {0.13278f, 0.77165f, 0.88580f},
        {0.12698f, 0.78037f, 0.87590f},
        {0.12151f, 0.78896f, 0.86581f},
        {0.11639f, 0.79740f, 0.85559f},
        {0.11167f, 0.80569f, 0.84525f},
        {0.10738f, 0.81381f, 0.83484f},
        {0.10357f, 0.82177f, 0.82437f},
        {0.10026f, 0.82955f, 0.81389f},
        {0.09750f, 0.83714f, 0.80342f},
        {0.09532f, 0.84455f, 0.79299f},
        {0.09377f, 0.85175f, 0.78264f},
        {0.09287f, 0.85875f, 0.77240f},
        {0.09267f, 0.86554f, 0.76230f},
        {0.09320f, 0.87211f, 0.75237f},
        {0.09451f, 0.87844f, 0.74265f},
        {0.09662f, 0.88454f, 0.73316f},
        {0.09958f, 0.89040f, 0.72393f},
        {0.10342f, 0.89600f, 0.71500f},
        {0.10815f, 0.90142f, 0.70599f},
        {0.11374f, 0.90673f, 0.69651f},
        {0.12014f, 0.91193f, 0.68660f},
        {0.12733f, 0.91701f, 0.67627f},
        {0.13526f, 0.92197f, 0.66556f},
        {0.14391f, 0.92680f, 0.65448f},
        {0.15323f, 0.93151f, 0.64308f},
        {0.16319f, 0.93609f, 0.63137f},
        {0.17377f, 0.94053f, 0.61938f},
        {0.18491f, 0.94484f, 0.60713f},
        {0.19659f, 0.94901f, 0.59466f},
        {0.20877f, 0.95304f, 0.58199f},
        {0.22142f, 0.95692f, 0.56914f},
        {0.23449f, 0.96065f, 0.55614f},
        {0.24797f, 0.96423f, 0.54303f},
        {0.26180f, 0.96765f, 0.52981f},
        {0.27597f, 0.97092f, 0.51653f},
        {0.29042f, 0.97403f, 0.50321f},
        {0.30513f, 0.97697f, 0.48987f},
        {0.32006f, 0.97974f, 0.47654f},
        {0.33517f, 0.98234f, 0.46325f},
        {0.35043f, 0.98477f, 0.45002f},
        {0.36581f, 0.98702f, 0.43688f},
        {0.38127f, 0.98909f, 0.42386f},
        {0.39678f, 0.99098f, 0.41098f},
        {0.41229f, 0.99268f, 0.39826f},
        {0.42778f, 0.99419f, 0.38575f},
        {0.44321f, 0.99551f, 0.37345f},
        {0.45854f, 0.99663f, 0.36140f},
        {0.47375f, 0.99755f, 0.34963f},
        {0.48879f, 0.99828f, 0.33816f},
        {0.50362f, 0.99879f, 0.32701f},
        {0.51822f, 0.99910f, 0.31622f},
        {0.53255f, 0.99919f, 0.30581f},
        {0.54658f, 0.99907f, 0.29581f},
        {0.56026f, 0.99873f, 0.28623f},
        {0.57357f, 0.99817f, 0.27712f},
        {0.58646f, 0.99739f, 0.26849f},
        {0.59891f, 0.99638f, 0.26038f},
        {0.61088f, 0.99514f, 0.25280f},
        {0.62233f, 0.99366f, 0.24579f},
        {0.63323f, 0.99195f, 0.23937f},
        {0.64362f, 0.98999f, 0.23356f},
        {0.65394f, 0.98775f, 0.22835f},
        {0.66428f, 0.98524f, 0.22370f},
        {0.67462f, 0.98246f, 0.21960f},
        {0.68494f, 0.97941f, 0.21602f},
        {0.69525f, 0.97610f, 0.21294f},
        {0.70553f, 0.97255f, 0.21032f},
        {0.71577f, 0.96875f, 0.20815f},
        {0.72596f, 0.96470f, 0.20640f},
        {0.73610f, 0.96043f, 0.20504f},
        {0.74617f, 0.95593f, 0.20406f},
        {0.75617f, 0.95121f, 0.20343f},
        {0.76608f, 0.94627f, 0.20311f},
        {0.77591f, 0.94113f, 0.20310f},
        {0.78563f, 0.93579f, 0.20336f},
        {0.79524f, 0.93025f, 0.20386f},
        {0.80473f, 0.92452f, 0.20459f},
        {0.81410f, 0.91861f, 0.20552f},
        {0.82333f, 0.91253f, 0.20663f},
        {0.83241f, 0.90627f, 0.20788f},
        {0.84133f, 0.89986f, 0.20926f},
        {0.85010f, 0.89328f, 0.21074f},
        {0.85868f, 0.88655f, 0.21230f},
        {0.86709f, 0.87968f, 0.21391f},
        {0.87530f, 0.87267f, 0.21555f},
        {0.88331f, 0.86553f, 0.21719f},
        {0.89112f, 0.85826f, 0.21880f},
        {0.89870f, 0.85087f, 0.22038f},
        {0.90605f, 0.84337f, 0.22188f},
        {0.91317f, 0.83576f, 0.22328f},
        {0.92004f, 0.82806f, 0.22456f},
        {0.92666f, 0.82025f, 0.22570f},
        {0.93301f, 0.81236f, 0.22667f},
        {0.93909f, 0.80439f, 0.22744f},
        {0.94489f, 0.79634f, 0.22800f},
        {0.95039f, 0.78823f, 0.22831f},
        {0.95560f, 0.78005f, 0.22836f},
        {0.96049f, 0.77181f, 0.22811f},
        {0.96507f, 0.76352f, 0.22754f},
        {0.96931f, 0.75519f, 0.22663f},
        {0.97323f, 0.74682f, 0.22536f},
        {0.97679f, 0.73842f, 0.22369f},
        {0.98000f, 0.73000f, 0.22161f},
        {0.98289f, 0.72140f, 0.21918f},
        {0.98549f, 0.71250f, 0.21650f},
        {0.98781f, 0.70330f, 0.21358f},
        {0.98986f, 0.69382f, 0.21043f},
        {0.99163f, 0.68408f, 0.20706f},
        {0.99314f, 0.67408f, 0.20348f},
        {0.99438f, 0.66386f, 0.19971f},
        {0.99535f, 0.65341f, 0.19577f},
        {0.99607f, 0.64277f, 0.19165f},
        {0.99654f, 0.63193f, 0.18738f},
        {0.99675f, 0.62093f, 0.18297f},
        {0.99672f, 0.60977f, 0.17842f},
        {0.99644f, 0.59846f, 0.17376f},
        {0.99593f, 0.58703f, 0.16899f},
        {0.99517f, 0.57549f, 0.16412f},
        {0.99419f, 0.56386f, 0.15918f},
        {0.99297f, 0.55214f, 0.15417f},
        {0.99153f, 0.54036f, 0.14910f},
        {0.98987f, 0.52854f, 0.14398f},
        {0.98799f, 0.51667f, 0.13883f},
        {0.98590f, 0.50479f, 0.13367f},
        {0.98360f, 0.49291f, 0.12849f},
        {0.98108f, 0.48104f, 0.12332f},
        {0.97837f, 0.46920f, 0.11817f},
        {0.97545f, 0.45740f, 0.11305f},
        {0.97234f, 0.44565f, 0.10797f},
        {0.96904f, 0.43399f, 0.10294f},
        {0.96555f, 0.42241f, 0.09798f},
        {0.96187f, 0.41093f, 0.09310f},
        {0.95801f, 0.39958f, 0.08831f},
        {0.95398f, 0.38836f, 0.08362f},
        {0.94977f, 0.37729f, 0.07905f},
        {0.94538f, 0.36638f, 0.07461f},
        {0.94084f, 0.35566f, 0.07031f},
        {0.93612f, 0.34513f, 0.06616f},
        {0.93125f, 0.33482f, 0.06218f},
        {0.92623f, 0.32473f, 0.05837f},
        {0.92105f, 0.31489f, 0.05475f},
        {0.91572f, 0.30530f, 0.05134f},
        {0.91024f, 0.29599f, 0.04814f},
        {0.90463f, 0.28696f, 0.04516f},
        {0.89888f, 0.27824f, 0.04243f},
        {0.89298f, 0.26981f, 0.03993f},
        {0.88691f, 0.26152f, 0.03753f},
        {0.88066f, 0.25334f, 0.03521f},
        {0.87422f, 0.24526f, 0.03297f},
        {0.86760f, 0.23730f, 0.03082f},
        {0.86079f, 0.22945f, 0.02875f},
        {0.85380f, 0.22170f, 0.02677f},
        {0.84662f, 0.21407f, 0.02487f},
        {0.83926f, 0.20654f, 0.02305f},
        {0.83172f, 0.19912f, 0.02131f},
        {0.82399f, 0.19182f, 0.01966f},
        {0.81608f, 0.18462f, 0.01809f},
        {0.80799f, 0.17753f, 0.01660f},
        {0.79971f, 0.17055f, 0.01520f},
        {0.79125f, 0.16368f, 0.01387f},
        {0.78260f, 0.15693f, 0.01264f},
        {0.77377f, 0.15028f, 0.01148f},
        {0.76476f, 0.14374f, 0.01041f},
        {0.75556f, 0.13731f, 0.00942f},
        {0.74617f, 0.13098f, 0.00851f},
        {0.73661f, 0.12477f, 0.00769f},
        {0.72686f, 0.11867f, 0.00695f},
        {0.71692f, 0.11268f, 0.00629f},
        {0.70680f, 0.10680f, 0.00571f},
        {0.69650f, 0.10102f, 0.00522f},
        {0.68602f, 0.09536f, 0.00481f},
        {0.67535f, 0.08980f, 0.00449f},
        {0.66449f, 0.08436f, 0.00424f},
        {0.65345f, 0.07902f, 0.00408f},
        {0.64223f, 0.07380f, 0.00401f},
        {0.63082f, 0.06868f, 0.00401f},
        {0.61923f, 0.06367f, 0.00410f},
        {0.60746f, 0.05878f, 0.00427f},
        {0.59550f, 0.05399f, 0.00453f},
        {0.58336f, 0.04931f, 0.00486f},
        {0.57103f, 0.04474f, 0.00529f},
        {0.55852f, 0.04028f, 0.00579f},
        {0.54583f, 0.03593f, 0.00638f},
        {0.53295f, 0.03169f, 0.00705f},
        {0.51989f, 0.02756f, 0.00780f},
        {0.50664f, 0.02354f, 0.00863f},
        {0.49321f, 0.01963f, 0.00955f},
        {0.47960f, 0.01583f, 0.01055f}
    };

    int index = static_cast<int>(x * 255.0f);
    return makeColor(turbo[index][0], turbo[index][1], turbo[index][2]);
  }
  std_msgs::msg::ColorRGBA colormapRainbow(float v)
  {
    v = std::clamp(v, 0.0f, 1.0f);
    float r = std::sin(2.0f * M_PI * v + 0.0f) * 0.5f + 0.5f;
    float g = std::sin(2.0f * M_PI * v + 2.0f * M_PI / 3.0f) * 0.5f + 0.5f;
    float b = std::sin(2.0f * M_PI * v + 4.0f * M_PI / 3.0f) * 0.5f + 0.5f;  
    return makeColor(r,g,b);
  }
  
  std_msgs::msg::ColorRGBA colormapParula(float v)
  {
    v = std::clamp(v, 0.0f, 1.0f);  
    // Parula approximation (6 key points from MATLAB)
    static const std::array<std::array<float, 3>, 6> parula_data = {{
      {0.2081f, 0.1663f, 0.5292f}, {0.2291f, 0.3220f, 0.5451f},
      {0.2669f, 0.4887f, 0.5561f}, {0.3052f, 0.6502f, 0.5653f},
      {0.5849f, 0.7823f, 0.4863f}, {0.9763f, 0.9831f, 0.0538f}
    }};
  
    float scaled = v * (parula_data.size() - 1);
    int idx1 = static_cast<int>(scaled);
    float frac = scaled - static_cast<float>(idx1);
  
    idx1 = std::max(0, std::min(idx1, static_cast<int>(parula_data.size()) - 2));
    int idx2 = std::min(idx1 + 1, static_cast<int>(parula_data.size()) - 1);
  
    float r = (1 - frac) * parula_data[idx1][0] + frac * parula_data[idx2][0];
    float g = (1 - frac) * parula_data[idx1][1] + frac * parula_data[idx2][1];
    float b = (1 - frac) * parula_data[idx1][2] + frac * parula_data[idx2][2];
  
    return makeColor(r, g, b);
  }
  // Type alias for colormap functions
  using ColormapFuncType = std::function<std_msgs::msg::ColorRGBA(float)>;

  // Static const array of colormap functions (Initialized once)
  // Ensure these enum values match the order and options in the constructor
  enum ColormapEnum { JET_CM = 0, HSV_CM, VIRIDIS_CM, RED_CM, GRAY_CM, TURBO_CM, RAINBOW_CM, PARULA_CM, NUM_COLORMAPS };
  enum ColorModeEnum { FLAT_COLOR = 0, INTENSITY_COLOR, CLUSTER_COLOR };
  static const std::array<ColormapFuncType, NUM_COLORMAPS> S_COLORMAP_FUNCTIONS = {
    colormapJet,  colormapHSV,   colormapViridis, colormapRed,
    colormapGray, colormapTurbo, colormapRainbow, colormapParula};
  


  DetectedObjectsWithFeatureDisplay::DetectedObjectsWithFeatureDisplay(
    const std::string & default_topic)
  : m_marker_common(this),
    m_line_width_property{"Line Width", 0.03, "Line width of object-shape", this},
    m_point_size_property{"Point Size", 0.1, "Point size of cluster", this},
    m_color_mode_property("Color Mode", 0, "Color mode of point cloud", this,SLOT(updateColormapPropertiesVisibility()),this),
    m_colormap_property{"Colormap", "Jet", "Colormap to use for intensity coloring", this},
    m_point_color_property{"Point Color", QColor(255, 0, 0), "Point color of object-shape", this},
    m_intensity_color_scale_max{
      "Intensity Threshold", 50.0, "Intensity threshold of point cloud", this},
    m_show_colorbar_property{
      "Show Colorbar", false, "Display the colormap colorbar in a separate window", this,SLOT(&DetectedObjectsWithFeatureDisplay::updateColorbarVisibility())},
  m_default_topic{default_topic}
{
  m_intensity_color_scale_max.setMin(1.0);
  m_intensity_color_scale_max.setMax(255.0);
  m_color_mode_property.addOption("Flat", 0);
  m_color_mode_property.addOption("Intensity", 1);
  m_color_mode_property.addOption("Cluster", 2);
  m_colormap_property.addOption("Jet", 0);
  m_colormap_property.addOption("HSV", 1);
  m_colormap_property.addOption("Viridis", 2);
  m_colormap_property.addOption("Red", 3);  
  m_colormap_property.addOption("Gray", 4);
  m_colormap_property.addOption("Turbo", 5);
  m_colormap_property.addOption("Rainbow", 6);
  m_colormap_property.addOption("Parula", 7); 
}

void DetectedObjectsWithFeatureDisplay::onInitialize()
{
  RosTopicDisplay::onInitialize();
  m_marker_common.initialize(this->context_, this->scene_node_);

  // Set topic message type and description
  QString message_type = QString::fromStdString(rosidl_generator_traits::name<DetectedObjectsWithFeature>());
  this->topic_property_->setMessageType(message_type);
  this->topic_property_->setDescription("Topic to subscribe to.");


  // Generate the colorbar for the colormap
  generateColorbar();
  // Create the colorbar widget
  m_colorbar_widget = new ColorbarWidget();

  //m_colorbar_widget->setWindowTitle("Colorbar");
  m_colorbar_widget->setColorbarImage(m_colorbar_image);  // Set the generated colorbar image
  m_colorbar_widget->setMinMax(0.0f, 1.0f);
  m_colorbar_widget->setWindowFlags(Qt::Tool);
  m_colorbar_widget->show();

  QObject::connect(
    &m_color_mode_property, &rviz_common::properties::Property::changed, this,
    [this]() { updateColormapPropertiesVisibility(); });
  QObject::connect(
    &m_show_colorbar_property, &rviz_common::properties::Property::changed, this,
    [this]() { updateColorbarVisibility(); });
  QObject::connect(
    &m_colormap_property, &rviz_common::properties::Property::changed, this,
    [this]() { updateColormapAndColorbar(); });
  QObject::connect(
    &m_intensity_color_scale_max, &rviz_common::properties::Property::changed, this,
    [this]() { updateColormapAndColorbar(); });
  updateColormapPropertiesVisibility();
  updateColorbarVisibility();
}
void DetectedObjectsWithFeatureDisplay::updateColormapAndColorbar()
{
  generateColorbar(); // Regenerate the image with the current colormap
  if (m_colorbar_widget) {
    m_colorbar_widget->setColorbarImage(m_colorbar_image);
    if (m_colorbar_widget->isVisible()) {
      m_colorbar_widget->update(); // Request a repaint of the colorbar widget
    }
  }
}
void DetectedObjectsWithFeatureDisplay::updateColormapPropertiesVisibility()
{
  std::cout<<"DetectedObjectsWithFeatureDisplay::updateColormapPropertiesVisibility()" << std::endl;
  int mode = m_color_mode_property.getOptionInt();
  bool is_intensity_mode_active = (mode == INTENSITY_COLOR);
  m_colormap_property.setHidden(!is_intensity_mode_active);
  m_show_colorbar_property.setHidden(!is_intensity_mode_active);
  m_intensity_color_scale_max.setHidden(!is_intensity_mode_active);
  m_point_color_property.setHidden(mode != FLAT_COLOR);
}
void DetectedObjectsWithFeatureDisplay::updateColorbarVisibility()
{
  std::cout<<"DetectedObjectsWithFeatureDisplay::updateColorbarVisibility()" << std::endl;
  if (!m_colorbar_widget) {
    return;
  }

  bool is_intensity_mode_active = (m_color_mode_property.getOptionInt() == INTENSITY_COLOR);
  bool should_show_widget = is_intensity_mode_active && m_show_colorbar_property.getBool();

  std::cout<<"DetectedObjectsWithFeatureDisplay::should_show_widget"<<should_show_widget << std::endl;
  if (should_show_widget) {
    // Ensure the colorbar image and labels are up-to-date before showing
    updateColormapAndColorbar(); // This generates the image and sets it
    //updateIntensityMax();      // This sets the min/max labels
    m_colorbar_widget->show();
  } else {
    m_colorbar_widget->hide();
  }
}
void DetectedObjectsWithFeatureDisplay::generateColorbar()
{
    int width = 20;  // Width of the colorbar
    int height = 200;  // Height of the colorbar
    auto colormap_fn = S_COLORMAP_FUNCTIONS[m_colormap_property.getOptionInt()];
    m_colorbar_image = QImage(width, height, QImage::Format_RGB32);
    for (int y = 0; y < height; ++y) {
        float norm = float(y) / (height - 1);
        std_msgs::msg::ColorRGBA color = colormap_fn(norm); 
        QRgb rgb = qRgb(color.r * 255, color.g * 255, color.b * 255);
        for (int x = 0; x < width; ++x) {
          m_colorbar_image.setPixelColor(x, y, rgb);
        }
      }
}
DetectedObjectsWithFeatureDisplay::~DetectedObjectsWithFeatureDisplay()
{
  delete m_colorbar_widget; // m_colorbar_widget_ is owned by this display
  // m_show_colorbar_property_ is also a child of 'this' and should be handled by Rviz's property system
  // or explicitly deleted if it wasn't parented or if Rviz doesn't manage it.
  // Typically, properties parented to 'this' are managed.
}

void DetectedObjectsWithFeatureDisplay::reset()
{
  RosTopicDisplay::reset();
  m_marker_common.clearMarkers();


  std::cout<<"DetectedObjectsWithFeatureDisplay::reset()" << std::endl;
}
void DetectedObjectsWithFeatureDisplay::processMessage(DetectedObjectsWithFeature::ConstSharedPtr msg)
{
  clear_markers();
  int id = 0;
  for (const auto & feature_object : msg->feature_objects) {
    const auto & cluster = feature_object.feature.cluster;
    if (cluster.width <= 0 || cluster.height <= 0) {      
      continue;  // Skip this cluster
    }
    // Create a marker to display the cluster point cloud
    double point_size = get_point_size();
    auto pointcloud_marker_ptr = std::make_shared<Marker>();
    pointcloud_marker_ptr->header = msg->header;
    pointcloud_marker_ptr->ns = "cluster_point_cloud";
    pointcloud_marker_ptr->id = static_cast<int>(id);
    pointcloud_marker_ptr->type = visualization_msgs::msg::Marker::POINTS;
    pointcloud_marker_ptr->action = visualization_msgs::msg::Marker::ADD;
    pointcloud_marker_ptr->pose.position.x = 0.0;
    pointcloud_marker_ptr->pose.position.y = 0.0;
    pointcloud_marker_ptr->pose.position.z = 0.0;
    pointcloud_marker_ptr->pose.orientation.w = 1.0;
    pointcloud_marker_ptr->scale.x = point_size; 
    pointcloud_marker_ptr->scale.y = point_size;
    pointcloud_marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
    pointcloud_marker_ptr->points.clear();
    pointcloud_marker_ptr->colors.clear();
    pointcloud_marker_ptr->points.reserve(cluster.width * cluster.height);
    pointcloud_marker_ptr->colors.reserve(cluster.width * cluster.height);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cluster, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cluster, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cluster, "z");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(cluster, "intensity");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      geometry_msgs::msg::Point point;
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;
      pointcloud_marker_ptr->points.push_back(point);
    }

    int mode = m_color_mode_property.getOptionInt();

    if (mode == 1) {
      int colormap_idx = m_colormap_property.getOptionInt();
      ColormapFuncType color_fn = S_COLORMAP_FUNCTIONS[JET_CM]; // Default
      if (colormap_idx >= 0 && static_cast<size_t>(colormap_idx) < S_COLORMAP_FUNCTIONS.size()) {
          color_fn = S_COLORMAP_FUNCTIONS[colormap_idx];
      }
      
      // Use intensity to color the points
      float max_intensity = m_intensity_color_scale_max.getFloat();

      for(; iter_intensity != iter_intensity.end(); ++iter_intensity) {
        float intensity_norm = static_cast<float>(*iter_intensity) / max_intensity;
        pointcloud_marker_ptr->colors.push_back(color_fn(intensity_norm));
      }
    } else if (mode == 2 /* Cluster */) {
      std_msgs::msg::ColorRGBA cluster_color = generateDistinctColor(id);
      for(; iter_intensity != iter_intensity.end(); ++iter_intensity) {
        pointcloud_marker_ptr->colors.push_back(cluster_color);
      }
    } else {
      // Use the default color
      std_msgs::msg::ColorRGBA color;
      color.r = get_point_color().redF();
      color.g = get_point_color().greenF();
      color.b = get_point_color().blueF();
      color.a = 1.0f;
      for(; iter_intensity != iter_intensity.end(); ++iter_intensity) {
        pointcloud_marker_ptr->colors.push_back(color);
      }
    }
    add_marker(pointcloud_marker_ptr);
    id++;
  }
}

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  autoware::rviz_plugins::object_detection::DetectedObjectsWithFeatureDisplay, rviz_common::Display)
