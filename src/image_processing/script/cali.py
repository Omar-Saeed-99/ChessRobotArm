#! /usr/bin/env python3
from roboflow import Roboflow
rf = Roboflow(api_key="API_KEY")
project = rf.workspace().project("chess-pieces-new")
model = project.version(19).model

# infer on a local image
# print(model.predict("IMG_20230224_215316jpg", confidence=40, overlap=30).json())

# visualize your prediction

model.predict("IMG_20230224_215316.jpg", confidence=40, overlap=30).save("prediction.jpg")

# infer on an image hosted elsewhere
# print(model.predict("URL_OF_YOUR_IMAGE", hosted=True, confidence=40, overlap=30).json())