# Running the FastAPI Tutorials

This directory contains a `main.py` that follows the official
[FastAPI tutorial](https://fastapi.tiangolo.com/tutorial/), covering first steps,
path and query parameters, request bodies, and query validations.

## How to run

If the `fastapi` CLI is not installed:

```
pip install "fastapi[standard]"
```

From this directory, start the development server:

```
fastapi dev main.py
```

## Where to see the result

The development server serves at `http://127.0.0.1:8000`. The interactive Swagger
UI documentation is available at `http://127.0.0.1:8000/docs`, and the ReDoc
documentation at `http://127.0.0.1:8000/redoc`.

Example endpoints defined in `main.py`:

- `http://127.0.0.1:8000/`
- `http://127.0.0.1:8000/items/{item_id}`
- `http://127.0.0.1:8000/models/{model_name}`
