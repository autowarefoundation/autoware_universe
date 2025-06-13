#!/usr/bin/env python3
import argparse
import sys

import yaml


def main():
    parser = argparse.ArgumentParser(
        description="Merge base and overlay .repos files with overlay taking precedence."
    )
    parser.add_argument("--base", required=True, help="Path to the base .repos file")
    parser.add_argument("--overlay", required=True, help="Path to the overlay .repos file")
    parser.add_argument(
        "--output",
        default="combined.repos",
        help="Path for the combined output file (default: combined.repos)",
    )
    args = parser.parse_args()

    try:
        with open(args.base, "r") as bf:
            base_data = yaml.safe_load(bf)
    except Exception as e:
        sys.exit(f"Error reading base file '{args.base}': {e}")

    try:
        with open(args.overlay, "r") as of:
            overlay_data = yaml.safe_load(of)
    except Exception as e:
        sys.exit(f"Error reading overlay file '{args.overlay}': {e}")

    if "repositories" not in base_data:
        sys.exit(f"Base file '{args.base}' is missing the 'repositories' key")
    if overlay_data and "repositories" not in overlay_data:
        sys.exit(f"Overlay file '{args.overlay}' is missing the 'repositories' key")

    # Merge: overlay entries override base entries
    merged_data = base_data.copy()
    merged_data["repositories"].update(overlay_data.get("repositories", {}))

    try:
        with open(args.output, "w") as cf:
            yaml.dump(merged_data, cf, default_flow_style=False)
    except Exception as e:
        sys.exit(f"Error writing to output file '{args.output}': {e}")

    print(f"Successfully merged into {args.output}")


if __name__ == "__main__":
    main()
