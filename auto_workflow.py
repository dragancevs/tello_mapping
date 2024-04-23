import Metashape
import os, sys, time

# This script is based on code from Agisoft https://github.com/agisoft-llc/metashape-scripts/blob/master/src/samples/general_workflow.py

# example command
# metashape.exe -r "C:\Users\Stefan\Documents\metashape\auto_workflow.py" "D:\fotogrammetrie_BP\Ruzova" "D:\fotogrammetrie_BP\Ruzova_model"


def find_files(folder, types):
    return [entry.path for entry in os.scandir(folder) if (entry.is_file() and os.path.splitext(entry.name)[1].lower() in types)]

# image_folder = r"D:\fotogrammetrie_BP\Ruzova"
# output_folder =r"D:\fotogrammetrie_BP\Ruzova_model"

if len(sys.argv) < 3:
    print("Usage: general_workflow.py <image_folder> <output_folder>")
    raise Exception("Invalid script arguments")

image_folder = sys.argv[1]
output_folder = sys.argv[2]

photos = find_files(image_folder, [".jpg", ".jpeg", ".tif", ".tiff"])

doc = Metashape.Document()
doc.save(output_folder + '/project.psx')

chunk = doc.addChunk()

chunk.addPhotos(photos)
doc.save()

print(str(len(chunk.cameras)) + " images loaded")

chunk.matchPhotos(keypoint_limit = 40000, tiepoint_limit = 10000, generic_preselection = True, reference_preselection = True)
doc.save()

chunk.alignCameras()
doc.save()

chunk.buildDepthMaps(downscale = 2, filter_mode = Metashape.MildFiltering)
doc.save()

chunk.buildModel(source_data = Metashape.DepthMapsData)
doc.save()

chunk.buildUV(page_count = 2, texture_size = 4096)
doc.save()

chunk.buildTexture(texture_size = 4096, ghosting_filter = True)
doc.save()
