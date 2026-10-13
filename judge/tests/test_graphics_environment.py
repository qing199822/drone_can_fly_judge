#!/usr/bin/env python3

import os
import pathlib
import subprocess
import tempfile
import unittest


PROJECT_ROOT = pathlib.Path(__file__).parents[1]
HELPER = PROJECT_ROOT / "scripts" / "graphics_environment.sh"
LAUNCHER = PROJECT_ROOT / "1.sh"


class GraphicsEnvironmentTest(unittest.TestCase):
    def test_imports_missing_display_from_desktop_session(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = pathlib.Path(temp_dir)
            authority = temp_path / "Xauthority"
            authority.touch()
            environ = temp_path / "environ"
            environ.write_bytes(
                b"DISPLAY=:7\0"
                + f"XAUTHORITY={authority}\0".encode()
                + b"XDG_RUNTIME_DIR=/run/user/1000\0"
            )

            command = (
                'unset DISPLAY XAUTHORITY XDG_RUNTIME_DIR; '
                'source "$1"; '
                'ensure_graphics_environment "$2"; '
                'printf "%s|%s|%s" "$DISPLAY" "$XAUTHORITY" "$XDG_RUNTIME_DIR"'
            )
            result = subprocess.run(
                ["bash", "-c", command, "bash", str(HELPER), str(environ)],
                check=True,
                capture_output=True,
                text=True,
                env={**os.environ, "PATH": os.environ["PATH"]},
            )

            self.assertEqual(
                f":7|{authority}|/run/user/1000",
                result.stdout,
            )

    def test_launcher_prepares_graphics_before_starting_gazebo(self):
        script = LAUNCHER.read_text()
        prepare = "ensure_graphics_environment"
        launch = 'roslaunch "$SIMULATION_LAUNCH"'

        self.assertIn(prepare, script)
        self.assertIn(launch, script)
        self.assertLess(script.index(prepare), script.index(launch))


if __name__ == "__main__":
    unittest.main()
