#!/usr/bin/env python3
"""Build, image, and run Roanix."""

import sys

sys.dont_write_bytecode = True

from tools.roanix_cli import main

if __name__ == "__main__":
    raise SystemExit(main())
