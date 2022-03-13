import zipfile, urllib.request, shutil

url = 'http://pix3d.csail.mit.edu/data/pix3d_full.zip'
file_name = 'pix3d_full.zip'

with urllib.request.urlopen(url) as response, open(file_name, 'wb') as out_file:
    shutil.copyfileobj(response, out_file)
    with zipfile.ZipFile(file_name) as zf:
        zf.extractall()