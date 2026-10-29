"""HTTP failure diagnostics must include the actual service rejection reason."""

import io
import unittest
import urllib.error
import urllib.parse
from unittest.mock import Mock, patch

import test_navigation  # Set up the same jev_api import path as other tests.
from tools.http_transport import JevTransport


class HttpErrorTests(unittest.TestCase):
    def test_persistent_response_keeps_error_body_and_closes_connection(self):
        transport = JevTransport()
        response = Mock(status=400)
        response.__enter__ = Mock(return_value=response)
        response.__exit__ = Mock(return_value=False)
        response.read.return_value = b'{"detail":{"error_type":"max_tokens_exceeded"}}'
        connection = Mock()
        connection.getresponse.return_value = response
        transport.connection = connection
        metrics = {}
        with patch.object(transport, '_resolve_route', return_value=(urllib.parse.urlsplit('https://example.test/v1'), None, None)), \
                patch.object(transport, '_prepare_connection'):
            with self.assertRaisesRegex(RuntimeError, 'HTTP 400.*max_tokens_exceeded'):
                transport.call({}, 'test-only', 'unused', metrics)
        self.assertEqual(metrics['http_status'], 400)
        self.assertIn('max_tokens_exceeded', metrics['http_error_body'])
        self.assertNotIn('test-only', str(metrics))
        response.read.assert_called_once_with(4096)
        connection.request.assert_called_once()
        connection.close.assert_called_once()

    def test_urllib_error_uses_the_same_diagnostics_without_retry(self):
        error = urllib.error.HTTPError('unused', 400, 'Bad Request', {},
                                       io.BytesIO(b'{"detail":{"error_type":"max_tokens_exceeded"}}'))
        transport = JevTransport()
        metrics = {}
        with patch.object(transport, '_call_urllib', side_effect=error) as call:
            with self.assertRaisesRegex(RuntimeError, 'HTTP 400.*max_tokens_exceeded'):
                transport.call({}, 'test-only', 'unused', metrics, 'urllib')
        self.assertEqual(metrics['http_status'], 400)
        call.assert_called_once()


if __name__ == '__main__':
    unittest.main()
