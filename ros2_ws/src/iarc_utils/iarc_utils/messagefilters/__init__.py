"""messagefilters: Classes for synchronizing and clamping multi-topic messages by timestamp."""

__all__ = ["LazyPX4MessageFilter", "PX4MessageFilter", "PX4MessageClamper"]


def __getattr__(name):
    """
    Lazy-import message filter classes on first access to reduce startup cost.

    Only PX4MessageFilter, LazyPX4MessageFilter, or PX4MessageClamper are recognized.
    """
    match name:
        case "LazyPX4MessageFilter":
            from .lazypx4messagefilter import LazyPX4MessageFilter

            return LazyPX4MessageFilter
        case "PX4MessageFilter":
            from .px4messagefilter import PX4MessageFilter

            return PX4MessageFilter
        case "PX4MessageClamper":
            from .px4messageclamper import PX4MessageClamper

            return PX4MessageClamper
        case _:
            raise AttributeError(f"module '{__name__}' has no attribute '{name}'")
