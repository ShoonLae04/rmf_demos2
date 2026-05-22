class ContractValidationError(ValueError):
    """Raised when inbound/outbound payload fails contract validation."""


class MappingError(ValueError):
    """Raised when payload cannot be mapped to RMF-compatible request."""


class DispatchError(RuntimeError):
    """Raised when RMF task dispatch fails."""
