__all__ = ['LazyPX4MessageFilter', 'PX4MessageFilter', 'PX4MessageClamper']

def __getattr__(name):
    """
    This function is called when an attribute is accessed that is not found in the module's namespace. It allows for lazy loading of the message filter classes, which can help reduce initial load time and memory usage if only one of the filters is needed.
    """
    match name:
        case 'LazyPX4MessageFilter':
            from .lazypx4messagefilter import LazyPX4MessageFilter
            return LazyPX4MessageFilter
        case 'PX4MessageFilter':
            from .px4messagefilter import PX4MessageFilter
            return PX4MessageFilter
        case 'PX4MessageClamper':
            from .px4messageclamper import PX4MessageClamper
            return PX4MessageClamper
        case _:
            raise AttributeError(f"module '{__name__}' has no attribute '{name}'")