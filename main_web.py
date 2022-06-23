#!/usr/bin/python3
# -*- coding:utf-8 -*-
import uvicorn
from fastapi import FastAPI
from fastapi import Request

from interface_ import *

app = FastAPI()


@app.post("/api/timekeeper")
async def home(request: Request):
    data = await request.json()
    return interface_func(track_id=int(data["InterfacePathParams"]["type"]-1),
                          user_workspace_dir=data["actualPath"])


if __name__ == '__main__':
    uvicorn.run(app, host='127.0.0.1', port=18888, debug=True)
