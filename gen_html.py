from glob import glob

def genfile(src,dest):
    with open(src,'r') as f:
        str = f.read()
        str = str.replace("\\","\\\\").replace("\n",'\\n').replace("\"","\\\"")
        with open(dest, 'w') as d:
            d.write(f'"{str}"')

for file in glob('html/*.html'):
    genfile(file, file + '.inl')