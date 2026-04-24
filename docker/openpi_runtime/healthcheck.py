#!/usr/bin/env python3
import os
import urllib.request


def main() -> int:
    port = int(os.environ.get("OPENPI_PORT", "8000"))
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/healthz", timeout=3) as resp:
            return 0 if resp.status == 200 else 1
    except Exception:
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
