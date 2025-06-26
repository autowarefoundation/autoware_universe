# combine-repos-action

## Description

**Combine Repos Action** is a GitHub Action designed to merge two `.repos` files—a base file and an overlay file—into a single file. The overlay file takes precedence over the base file when duplicate repository entries exist. This is especially useful for projects that need to dynamically combine configuration files for dependency management or CI workflows.

## Usage

Below is an example of how to include it in your workflow:

```yaml
jobs:
  combine:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout repository
        uses: actions/checkout@v4

      - name: Combine Repos Files
        uses: ./.github/actions/combine-repos-action
        with:
          base_file: build_depends_humble.repos
          overlay_file: build_depends_nightly.repos
          output_file: build_depends.repos
```

In this example:

- The action reads the `build_depends_humble.repos` file and the `build_depends_nightly.repos` file.
- It merges them with overlay file taking precedence.
- The resulting file is saved as `build_depends.repos` (or a custom filename if specified).

## Inputs

| Input          | Description                       | Required | Default          |
| -------------- | --------------------------------- | -------- | ---------------- |
| `base_file`    | Path to the base `.repos` file    | Yes      | -                |
| `overlay_file` | Path to the overlay `.repos` file | Yes      | -                |
| `output_file`  | Path for the combined output file | No       | `combined.repos` |

## Outputs

This action creates or updates the file specified by the `output_file` input with the combined contents of the two `.repos` files. The final file's content is also echoed to the workflow logs for easy verification.
