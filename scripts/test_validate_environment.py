"""Software checks do not require selecting a drone or configuring its address."""

import contextlib
import io
import unittest
from unittest.mock import patch

import validate_environment


class EnvironmentTests(unittest.TestCase):
    def test_default_checks_only_software(self):
        output = io.StringIO()
        with (
            patch('sys.argv', ['validate_environment.py']),
            patch.object(validate_environment, 'check_imports', return_value=True) as imports,
            patch.object(validate_environment, 'check_radio') as radio,
            patch.object(validate_environment, 'check_docker') as docker,
            contextlib.redirect_stdout(output),
        ):
            self.assertEqual(validate_environment.main(), 0)
        imports.assert_called_once_with()
        radio.assert_not_called()
        docker.assert_not_called()
        self.assertIn('Environment ready.', output.getvalue())
        self.assertNotIn('CF_URI', output.getvalue())
        self.assertNotIn('tl-config.env', output.getvalue())

    def test_optional_radio_check_does_not_select_a_drone(self):
        with (
            patch('sys.argv', ['validate_environment.py', '--radio']),
            patch.object(validate_environment, 'check_imports', return_value=True),
            patch.object(validate_environment, 'check_radio', return_value=False) as radio,
            contextlib.redirect_stdout(io.StringIO()),
        ):
            self.assertEqual(validate_environment.main(), 1)
        radio.assert_called_once_with()
