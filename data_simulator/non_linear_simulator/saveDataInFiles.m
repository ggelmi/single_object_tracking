function file = saveDataInFiles(measData)
  
  n = length(measData);
  
  for i = 1:n
    detections = measData{i,1}
    name = "detection_file_";
    filename = strcat(name,num2str(i))
    path = strcat("/home/guuto/projects/single_object_tracking/data_simulator/data/",filename)
    fid = fopen (path, "w+");
    for j=1:size(detections, 1)
    fprintf(fid, '%f ', detections(j,:));
    fprintf(fid, '\n');
    end
    fclose(fid);
    
  end
  
end
