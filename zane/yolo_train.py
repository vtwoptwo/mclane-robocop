#from ultralytics import YOLO

#model = YOLO('yolov8n.yaml')  # build a new model from YAML
#model = YOLO('yolov8n.pt')  # load a pretrained model (recommended for training)
#model = YOLO('yolov8n.yaml').load('yolov8n.pt')  # build from YAML and transfer weights

# Train the model
#results = model.train(data='data.yaml', epochs=100, imgsz=640)

from ultralytics import YOLO
import cv2

def train_yolo():
    # Initialize and load the model
    model = YOLO('yolov8n.pt')  # load a pretrained model (recommended for training)

    # Train the model
    results = model.train(data='../datasets/fruits/data.yaml', epochs=20, imgsz=640, save_dir='fruits')
    return results

def test_model(path_to_model):
    # Load the trained YOLO model
    model = YOLO(path_to_model)

    # Start video capture from the camera
    # Note: The camera index may vary. Usually, 0 works for the default camera,
    # but if it doesn't work, you might need to try different indices like 1, 2, etc.
    cap = cv2.VideoCapture(0)  

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Perform detection using the YOLO model
        results = model(frame)
        # Inspect the structure of results
        #print(f"Results type: {type(results)}, Length: {len(results)}")
        #if len(results) > 0:
            #print(f"First element type: {type(results[0])}")

        # Assuming each element in results is a detection
        for detection in results:
            print(type(detection))  # Print and inspect the structure of a single detection


        # Process results here (e.g., draw bounding boxes, labels)
        # This might involve converting the results to a suitable format
        # and iterating over detections to draw on the frame

        # Display the processed frame
        cv2.imshow('YOLO Object Detection', frame)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture when everything is done
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    train_results = train_yolo()
    #test_model("./runs/detect/train3/weights/best.pt")
