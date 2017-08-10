#include <stdio.h>
#include <KudanSLAM.hpp>

int main(int argc, char **argv)
{
    printf("This is Kudan SLAM: %s \n", KudanSLAM::version().c_str());
    
    KudanSLAM slam;
    
    slam.setDescriptorMode();
    int w = 640;
    int h = 480;
    slam.setCameraCalibration(w, h, 500, 500, w / 2, h / 2);
    
    KudanVector2 f = slam.getFocalLength();
    
    printf("Focal length is %f, %f \n", f.x, f.y);
    
    
    KudanMatrix3 K = slam.getCalibrationMatrix();
    printf("K = \n");
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%f, ", K.data[i * 3 + j]);
        }
        
        printf("\n");
    }
    
    printf("\n");
    
    printf("byee\n");
}
