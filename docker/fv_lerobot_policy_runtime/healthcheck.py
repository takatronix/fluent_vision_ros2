#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import sys
import urllib.request


def main() -> int:
    port = int(os.environ.get("DPEX_PORT", "8010"))
    url = f"http://127.0.0.1:{port}/healthz"
    try:
        with urllib.request.urlopen(url, timeout=5) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
    except Exception as exc:
        print(f"healthcheck failed: {exc}", file=sys.stderr)
        return 1
    return 0 if payload.get("ok", False) else 1


if __name__ == "__main__":
    raise SystemExit(main())
