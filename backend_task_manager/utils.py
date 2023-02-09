import os
from io import BytesIO
import zipfile


def check_parameters(*expression):
    assert all(expression), "Missing parameters"


def get_dir_as_zip(dir_path):
    memory_file = BytesIO()

    with zipfile.ZipFile(memory_file, 'w', zipfile.ZIP_DEFLATED) as zf:
        zipdir(dir_path, zf)

    memory_file.seek(0)

    return memory_file


def zipdir(path, ziph):
    # ziph is zipfile handle
    for root, dirs, files in os.walk(path):
        for file in files:
            ziph.write(os.path.join(root, file), 
                       os.path.relpath(os.path.join(root, file), 
                                       os.path.join(path, '..')))

