# lint-system-design-format

## Description

**Lint Autoware System Design Format Action** is a GitHub Action designed to lint autoware system design format YAML files (`.node.yaml`, `.module.yaml`, `.system.yaml`, `.parameter_set.yaml`) using the linter from the `autoware_system_designer` repository.

This action:

1. Checks out the `autoware_system_designer` repository
2. Sets up Python and installs dependencies
3. Finds all design format files in the specified path
4. Runs the linter and reports errors in GitHub Actions format

## Usage

Below is an example of how to include it in your workflow:

```yaml
jobs:
  lint:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout repository
        uses: actions/checkout@v4

      - name: Lint system design format files
        uses: ./.github/actions/lint-system-design-format
        with:
          design_files_path: "common/autoware_universe_designs/design"
```

## Inputs

| Input                              | Description                                       | Required | Default                                                                |
| ---------------------------------- | ------------------------------------------------- | -------- | ---------------------------------------------------------------------- |
| `design_files_path`                | Path to directory containing design files to lint | No       | `'.'`                                                                  |
| `autoware_system_designer_repo`    | Repository URL for autoware_system_designer       | No       | `'https://github.com/autowarefoundation/autoware_system_designer.git'` |
| `autoware_system_designer_version` | Version/branch of autoware_system_designer to use | No       | `'main'`                                                               |
| `python_version`                   | Python version to use                             | No       | `'3.10'`                                                               |

## Outputs

This action outputs linting results in GitHub Actions format. Errors and warnings will appear as annotations in the GitHub Actions UI. The action will fail if any linting errors are found.
