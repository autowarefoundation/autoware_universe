# image_transport_decompressor

## Purpose

The `image_transport_decompressor` is a node that decompresses images.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                       | Type                                | Description      |
| -------------------------- | ----------------------------------- | ---------------- |
| `~/input/compressed_image` | `sensor_msgs::msg::CompressedImage` | compressed image |

### Output

| Name                 | Type                      | Description        |
| -------------------- | ------------------------- | ------------------ |
| `~/output/raw_image` | `sensor_msgs::msg::Image` | decompressed image |

## Parameters

{{ json_to_markdown("sensing/autoware_image_transport_decompressor/schema/image_transport_decompressor.schema.json") }}

## Assumptions / Known limits

The supported case is an 8-bit RGB or BGR camera with `encoding` set to `rgb8` or `bgr8`. Anything
else still decodes and publishes, but the result is wrong in one of two ways, because the decode
always yields 8-bit BGR with three channels whatever the camera sent.

- **Another camera, with `encoding: rgb8` or `bgr8`.** The message is well formed, so no consumer
  complains, but the pixels are no longer what the camera measured.
- **Any other `encoding`.** The published `encoding` is the one the sender named, and the payload no
  longer matches it. `cv_bridge` throws on the 16-bit encodings and misreads the rest in silence.

Specifically:

| Camera image       | `encoding: rgb8` or `bgr8`         | any other `encoding`                |
| ------------------ | ---------------------------------- | ----------------------------------- |
| `rgb8`, `bgr8`     | as sent                            | as sent                             |
| `rgba8`, `bgra8`   | alpha dropped                      | alpha replaced by 255               |
| `mono8`            | one channel copied into three      | **three channels named `mono8`**    |
| `mono16`           | upper 8 bits, copied into three    | **three channels named `mono16`**   |
| `rgb16`, `bgr16`   | upper 8 bits                       | **8-bit samples named 16-bit**      |
| `rgba16`, `bgra16` | upper 8 bits, alpha dropped        | **8-bit samples named 16-bit**      |
| `bayer_rggb8`      | pattern copied, no color recovered | **three channels named as a Bayer** |
| `yuv422`           | as sent                            | **BGR pixels named `yuv422`**       |

The bold cells are the second case: the payload does not have the encoding it is published under.

An undecodable payload is dropped and reported with `RCLCPP_ERROR`.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
