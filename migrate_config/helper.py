def recursive_json_iterator(obj, k=None, separator='/'):
    """Iterates key value pairs recursively.

    This is a generator function.

    Args:
        obj (dict): Dictionary to iterate.

    Returns:
        generator: A generator function yielding key/value pair.

    """
    for k0, v0 in obj.items():
        if isinstance(v0, dict):
            for k1, v1 in recursive_json_iterator(v0, k0):
                yield (k1, v1) if k is None else (k + separator + k1, v1)
        else:
            yield (k0, v0) if k is None else (k + separator + k0, v0)


def getshape(d):
    if isinstance(d, dict):
        return {k: getshape(d[k]) for k in d}
    elif isinstance(d, list):
        return []
    else:
        # Replace all non-dict values with None.
        return None
