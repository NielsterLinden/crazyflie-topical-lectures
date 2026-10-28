"""Offline tests for the version policy shared by both setup scripts."""

import unittest

from check_python_version import check_version


class PythonVersionTests(unittest.TestCase):
    def test_tested_version_needs_no_warning(self):
        self.assertIsNone(check_version((3, 12)))

    def test_newer_versions_warn_but_are_allowed(self):
        for minor in (13, 14, 15):
            with self.subTest(minor=minor):
                warning = check_version((3, minor))
                self.assertIn("WARNING", warning)
                self.assertIn(f"3.{minor}", warning)
                self.assertIn("untested", warning)

    def test_older_versions_are_rejected(self):
        for version in ((2, 7), (3, 9), (3, 11)):
            with self.subTest(version=version):
                with self.assertRaisesRegex(ValueError, "3.12 or newer"):
                    check_version(version)


if __name__ == "__main__":
    unittest.main()
