from ultralytics import YOLO
import sys

def export_model(input_model_path, output_format):
    # Load the model (YOLO will automatically know if it's a .yaml or .pt)
    model = YOLO(input_model_path)

    # Export to the specified format
    success = model.export(format=output_format)

    if success:
        print(f"Model exported successfully to {output_format} format!")
    else:
        print("Model export failed.")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python export_model.py <input_model_path> <output_format>")
        print("Example: python export_model.py yolo11n.pt onnx")
        sys.exit(1)

    input_model_path = str(sys.argv[1])
    output_format = sys.argv[2]

    export_model(input_model_path, output_format)
