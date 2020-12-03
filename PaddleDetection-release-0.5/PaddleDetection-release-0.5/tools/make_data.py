import os

total_xml=[]
xmlfilepath = '../mydata/Annotations'
imgfilepath = '../mydata/JPEGImages'
xmlfilelist = os.listdir(xmlfilepath)
imgfilelist  = os.listdir(imgfilepath)
write_file = '../mydata/ImageSets/'
for j in range(3):
    xmlfile = xmlfilepath +'/'+xmlfilelist[j]
    imgfile = imgfilepath +'/'+imgfilelist[j]
    imgfiles = os.listdir(imgfile)
    xmlfiles = os.listdir(xmlfile)
    if(len(imgfiles) == len(xmlfiles)):
        lines = []
        for i in range(len(imgfiles)):
            line = '../JPEGImages/'+ imgfilelist[j] + '/' + imgfiles[i]  + ' '+ '../Annotations/' + xmlfilelist[j] + '/' + os.path.splitext(imgfiles[i])[0] +'.xml'                    # 循环读取路径下的文件并筛选输出
            lines.append(line)
    with open(write_file+xmlfilelist[j]+'.txt','w') as f:
        for i in range(len(lines)):
            f.write(lines[i])
            f.write('\n')