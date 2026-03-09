import itertools
from datetime import datetime

_counter = itertools.count(1)

def new_id(prefix):
    ts = datetime.now().strftime("%Y%m%d%H%M%S")
    return f"{prefix}_{ts}_{next(_counter):04d}"


if __name__ == "__main__":
    print(new_id("moveJ"))
    print(new_id("moveJ"))
    print(new_id("moveL"))