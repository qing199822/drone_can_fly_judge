#!/usr/bin/env python3

import pathlib
import re
import unittest


class GimbalLauncherTest(unittest.TestCase):
    def test_all_gimbal_workers_use_python3(self):
        launcher = (
            pathlib.Path(__file__).resolve().parents[1]
            / "src"
            / "gimbal"
            / "multi_gimbal_control.sh"
        ).read_text(encoding="utf-8")

        self.assertNotRegex(launcher, re.compile(r"^\s*python\s", re.MULTILINE))
        self.assertRegex(launcher, re.compile(r"^\s*python3\s", re.MULTILINE))


if __name__ == "__main__":
    unittest.main()
