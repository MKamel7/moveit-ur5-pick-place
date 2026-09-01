#!/usr/bin/env python3
"""Anonymous clients cannot command this cell.

The server exposes a writable Safety object (EStop, GuardClosed, HumanPresent,
Reset) on 0.0.0.0:4840. It previously accepted anonymous connections, so
anyone who could reach the port could assert an emergency stop, clear a guard,
or fake a human presence signal on a running cell. Nothing failed when that
was true, which is why it stayed true.

These assertions run without ROS, asyncua, a robot or a server, so they belong
in the fast gate and will fail the moment somebody reintroduces anonymous
access or writes a password into the repository.
"""

import re
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
from armik_moveit.opcua_security import (  # noqa: E402
    Credentials,
    authenticate,
    certificate_paths,
)


def test_anonymous_is_refused():
    # This is how an anonymous OPC UA session arrives at the user manager.
    assert authenticate(None, None) is False
    assert authenticate('', '') is False
    assert authenticate('supervisor', None) is False
    assert authenticate(None, 'anything') is False


def test_the_wrong_password_is_refused():
    account = Credentials()
    assert authenticate(account.username, 'wrong') is False
    assert authenticate('operator', account.password) is False
    assert authenticate(account.username, account.password[:-1]) is False


def test_the_right_credential_is_accepted():
    account = Credentials()
    assert authenticate(account.username, account.password) is True


def test_a_non_string_token_cannot_slip_through():
    # A malformed or unexpected identity token must not compare equal to
    # anything, and must not raise either.
    for bad in (0, 1, [], {}, object(), True):
        assert authenticate(bad, bad) is False


def test_no_password_is_written_down_in_this_repository():
    # The generated default exists precisely so that a committed credential
    # never does. If someone adds one, this fails.
    account = Credentials()
    suspicious = re.compile(
        r'CELL_OPCUA_PASSWORD\s*[=:]\s*["\'][^"\']+["\']')
    for path in PKG.rglob('*.py'):
        text = path.read_text(encoding='utf-8', errors='ignore')
        assert not suspicious.search(text), (
            f'{path} appears to hardcode an OPC UA password')
        assert account.password not in text or path.name == 'opcua_security.py'


def test_encryption_is_off_unless_a_real_keypair_is_supplied(monkeypatch):
    monkeypatch.delenv('CELL_OPCUA_CERT', raising=False)
    monkeypatch.delenv('CELL_OPCUA_KEY', raising=False)
    assert certificate_paths() == (None, None)
    # A path that does not exist must not be reported as usable, or the server
    # would claim an encrypted channel it does not have.
    monkeypatch.setenv('CELL_OPCUA_CERT', '/nonexistent/cert.pem')
    monkeypatch.setenv('CELL_OPCUA_KEY', '/nonexistent/key.pem')
    assert certificate_paths() == (None, None)


def test_the_generated_password_is_not_guessable():
    account = Credentials()
    assert len(account.password) >= 20
    assert account.password != account.username
    assert Credentials().password == account.password, (
        'the password must be stable within a process, or a client and server '
        'in the same process cannot agree')
