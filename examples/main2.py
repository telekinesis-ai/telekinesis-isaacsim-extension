from pydantic import BaseModel
from fastapi import FastAPI
from enum import Enum

app = FastAPI()

# https://fastapi.tiangolo.com/tutorial/first-steps/


@app.get("/")
async def root():
    return {"message": "Hello World"}


# https://fastapi.tiangolo.com/tutorial/path-params/
@app.get("/items/{item_id}")
async def read_item(item_id: int):
    return {"item_id": item_id}


# https://fastapi.tiangolo.com/tutorial/query-params/
class ModelName(str, Enum):
    alexnet = "alexnet"
    resnet = "resnet"
    lenet = "lenet"


@app.get("/models/{model_name}")
async def get_model(model_name: ModelName):
    if model_name is ModelName.alexnet:
        return {"model_name": model_name, "message": "Deep Learning FTW!"}

    if model_name.value == "lenet":
        return {"model_name": model_name, "message": "LeCNN all the images"}

    return {"model_name": model_name, "message": "Have some residuals"}


# https://fastapi.tiangolo.com/tutorial/body/#create-your-data-model


class Item(BaseModel):
    name: str
    description: str | None = None
    price: float
    tax: float | None = None


@app.post("/items/")
async def create_item(item: Item):
    item_dict = item.model_dump()
    if item.tax is not None:
        price_with_tax = item.price + item.tax
        item_dict.update({"price_with_tax": price_with_tax})
    return item_dict

@app.put("/items/{item_id}")
async def update_item(item_id: int, item: Item, q: str | None = None):
    result = {"item_id": item_id, **item.model_dump()}
    if q:
        result.update({"q": q})
    return result



#https://fastapi.tiangolo.com/tutorial/query-params-str-validations/
from typing import Annotated

from fastapi import FastAPI, Query

@app.get("/items/")
async def read_items(q: Annotated[str | None, Query(max_length=50)] = None):
    results = {"items": [{"item_id": "Foo"}, {"item_id": "Bar"}]}
    if q:
        results.update({"q": q})
    return results