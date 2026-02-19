class RBY1SDKError(Exception):
    """Base exception for rb_sdk rby1 adapter."""


class RBY1SDKImportError(RBY1SDKError):
    """Raised when upstream rby1 sdk is not installed."""


class RBY1SDKConnectionError(RBY1SDKError):
    """Raised when rby1 connection fails."""
