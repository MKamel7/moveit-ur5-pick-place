#!/usr/bin/env python3
"""Who is allowed to command this cell over OPC UA.

WHY THIS EXISTS. The server used to be a bare `Server()` on
`opc.tcp://0.0.0.0:4840/cell/` with no security policy and no user manager,
while calling `set_writable()` on the Safety object's EStop, GuardClosed,
HumanPresent and Reset. Anonymous OPC UA is the default in almost every
demonstration, and here it meant anyone who could reach the port could write
the safety inputs of a cell whose entire point is functional safety. A
supervisory interface that anyone on the network can command is not a
supervisory interface, it is an actuator with a nice browse tree.

The sibling project `virtual-production-cell` already solved this properly
(`src/vpc/opcua.py`), and this module follows the same rules:

  - Anonymous is REFUSED rather than discouraged. It is not in the offered
    identity tokens, so it is not on the menu at all.
  - There is NO default password anywhere in this repository and no file to
    put one in. A credential in source control is a credential every clone and
    every future reader has. If `CELL_OPCUA_PASSWORD` is unset, one is
    generated for the run and printed at startup, so the insecure option does
    not exist rather than merely being frowned upon.

The credential check is kept here, free of ROS and of asyncua imports, so it
can be tested without a graph, a robot or a running server.
"""

import hmac
import os
import secrets
from dataclasses import dataclass, field

#: Evaluated once per process so a server and a client in the same process
#: agree without either being told. Across processes there is nothing to agree
#: on, which is why the server prints it at startup.
_PASSWORD = os.environ.get("CELL_OPCUA_PASSWORD") or secrets.token_urlsafe(18)

#: True when the password above was generated rather than supplied, so the
#: server can say so instead of leaving somebody guessing.
PASSWORD_WAS_GENERATED = "CELL_OPCUA_PASSWORD" not in os.environ


@dataclass(frozen=True)
class Credentials:
    """The one account this server accepts.

    Set `CELL_OPCUA_USER` and `CELL_OPCUA_PASSWORD` to choose them. Neither has
    a value written down anywhere in this repository.
    """

    username: str = field(
        default_factory=lambda: os.environ.get("CELL_OPCUA_USER", "supervisor"))
    password: str = field(default_factory=lambda: _PASSWORD)


def authenticate(username, password, account=None):
    """Is this the one account the cell accepts?

    Anonymous arrives here as `None`/`None` and is refused, which is the whole
    point: the caller must not be able to reach a writable EStop without
    presenting a credential.

    Compared with `hmac.compare_digest` rather than `==` so the check does not
    leak the credential one character at a time through its own timing.
    """
    account = account or Credentials()
    if not isinstance(username, str) or not isinstance(password, str):
        return False            # anonymous, or a malformed token
    user_ok = hmac.compare_digest(username, account.username)
    pass_ok = hmac.compare_digest(password, account.password)
    return user_ok and pass_ok


def certificate_paths():
    """The certificate and key to sign and encrypt with, if they were given.

    Returns `(cert, key)` or `(None, None)`. Transport encryption needs a
    keypair, and generating one belongs in a shared library rather than being
    copied a third time into this repository, so this reads the pair from the
    environment and the server says plainly which mode it ended up in.
    """
    cert = os.environ.get("CELL_OPCUA_CERT")
    key = os.environ.get("CELL_OPCUA_KEY")
    if cert and key and os.path.isfile(cert) and os.path.isfile(key):
        return cert, key
    return None, None
