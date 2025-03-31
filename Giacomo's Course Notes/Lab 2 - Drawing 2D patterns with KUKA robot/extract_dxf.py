import ezdxf
import pandas as pd
import tkinter as tk
from tkinter import filedialog
import os

def extract_dxf_points(dxf_path, output_excel_path):
    # Load the DXF file
    doc = ezdxf.readfile(dxf_path)
    modelspace = doc.modelspace()

    # Prepare a list for storing coordinates
    points = []

    print("Entities found in DXF:")
    
    # Extract points from the DXF
    for entity in modelspace:
        print(entity.dxftype())  # Debugging: Print entity type
        
        if entity.dxftype() == "POINT":
            points.append([entity.dxf.location.x, entity.dxf.location.y])

    # If no points were found, exit
    if not points:
        print("No valid points extracted from the DXF file.")
        return None

    # Convert to DataFrame
    df = pd.DataFrame(points, columns=["X (mm)", "Y (mm)"])

    # Save to Excel
    df.to_excel(output_excel_path, index=False)
    print(f"Coordinates saved to: {output_excel_path}")

    return df

# GUI File Picker
root = tk.Tk()
root.withdraw()
dxf_file = filedialog.askopenfilename(title="Select a DXF file", filetypes=[("DXF files", "*.dxf")])

if dxf_file:
    output_folder = os.path.dirname(dxf_file)
    output_excel = os.path.join(output_folder, "DXF_Coordinates.xlsx")

    # Extract points and save to Excel
    df = extract_dxf_points(dxf_file, output_excel)

    # Show the result
    if df is not None:
        print(df)
    else:
        print("No valid coordinates found in the DXF file.")
else:
    print("No file selected.")
