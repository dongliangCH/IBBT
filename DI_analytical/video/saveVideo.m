writerObj=VideoWriter('test.avi'); 
writerObj.FrameRate = 1;
open(writerObj); 
for i = 1:size(im,2)
    [A,map] = rgb2ind(im{i},256);
    writeVideo(writerObj,im2frame(A, map));
end
close(writerObj);   