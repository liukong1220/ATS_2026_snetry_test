#!/usr/bin/env python3
"""Host-process fixtures only: no ROS participant or DDS traffic is created."""
import os
import signal
import subprocess
import sys
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

import nav_domain_guard as guard


class DomainGuardTest(unittest.TestCase):
    def test_exclusions_and_rtps_port_range(self):
        for domain in (-1, 233, 5, 77, 91, 92):
            with self.assertRaises(ValueError):
                guard.validate_domain(domain)
        self.assertEqual(guard.validate_domain(232), 232)
        self.assertEqual(guard.validate_domain(0), 0)
        base = 7400 + 250 * 20
        text = "\n".join("UNCONN 0 0 0.0.0.0:%d 0.0.0.0:*" % port
                         for port in (base - 1, base, base + 11, base + 249, base + 250))
        self.assertEqual(guard.domain_ports(20, text), [base, base + 11, base + 249])

    def test_cleanup_requires_uid_domain_and_kernel_start_identity(self):
        identity = dict(pid=1234, start_ticks=55, domain="20", uid=os.getuid())
        with patch.object(guard, "process_identity", return_value=identity):
            self.assertTrue(guard.same_process(identity))
            for changed in (dict(identity, start_ticks=54), dict(identity, domain="21"),
                            dict(identity, uid=os.getuid() + 1)):
                with patch.object(guard, "process_identity", return_value=changed):
                    self.assertFalse(guard.same_process(identity))

    def test_owned_cleanup_leaves_unlisted_process_alive(self):
        owned = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"],
                                 env=dict(os.environ, ROS_DOMAIN_ID="20"))
        other = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"],
                                 env=dict(os.environ, ROS_DOMAIN_ID="20"))
        try:
            identity = guard.process_identity(owned.pid)
            self.assertEqual(identity["domain"], "20")
            self.assertEqual(guard.cleanup_owned([identity]), [])
            owned.wait(timeout=3)
            self.assertIsNone(other.poll(), "cleanup signaled an unlisted same-domain process")
        finally:
            for child in (owned, other):
                if child.poll() is None:
                    child.send_signal(signal.SIGTERM)
                child.wait(timeout=3)


if __name__ == "__main__":
    unittest.main()
