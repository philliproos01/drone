import cv2
import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras.layers import DepthwiseConv2D

# Patch DepthwiseConv2D.from_config to remove unsupported 'groups' safely
original_from_config = DepthwiseConv2D.from_config

def patched_from_config(*args, **kwargs):
    # The config can be in args[0] or kwargs['config']
    config = None
    if args:
        config = args
    elif 'config' in kwargs:
        config = kwargs['config']

    if isinstance(config, dict) and 'groups' in config:
        config.pop('groups')

    return original_from_config(*args, **kwargs)

DepthwiseConv2D.from_config = classmethod(patched_from_config)

# Load your model (adjust 'Model.h5' to your model path)
model = load_model('Model.h5')

# Replace with your actual plant class names
class_names = ['PlantA', 'PlantB', 'PlantC']

IMG_SIZE = 224  # Adjust based on model expected input size

def preprocess_frame(frame):
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (IMG_SIZE, IMG_SIZE))
    img = img / 255.0
    img = np.expand_dims(img, axis=0)
    return img

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        input_img = preprocess_frame(frame)
        preds = model.predict(input_img)
        class_idx = np.argmax(preds)
        class_confidence = preds[0][class_idx]

        label = f"{class_names[class_idx]}: {class_confidence*100:.2f}%"
        cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow('Plant Recognition', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
