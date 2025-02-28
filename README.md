Development installation using uv
=================================

- Clone the repository

    ```console
    git clone git@github.com:carnegie-observatories/galil.git
    ```

- Create a virtual environment

    ```console
    cd galil
    uv venv

- Activate the virtual environment

    ```console
    source .venv/bin/activate
    ```

- Install the library

    ```console
    uv sync
    ```

Example usage
============

```python
from galil.galil import Galil
g = Galil("10.7.45.146")
g.move_to_angle(0)
````