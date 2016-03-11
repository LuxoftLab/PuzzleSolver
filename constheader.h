#ifndef CONSTHEADER
#define CONSTHEADER

const char* originalPath = "C://OpenCV-2.4.9//opencv//sources//samples//c//airplane.jpg";
const char* maskPath = "C://Users//KirillUser//Documents//CannyStill1//new_mask_airplane.jpg";
const char* resultPath = "C://Users//KirillUser//Documents//CannyStill1//result_airplane.jpg";
const char* generatedPuzzlesPath = "C://Users//KirillUser//Documents//CannyStill1//generatedImage_airplane.jpg";
const char* generatedPuzzlesParts = "C://Users//KirillUser//Documents//CannyStill1//airplane_part_";

const int IMG_SZ_WIDTH = 512;
const int IMG_SZ_HEIGHT = 512;

const int KOEF_SHUFFLE = 20;
const int KOEF_SET_MASK = 2;
const int FLOOD_CONST_X = 70;
const int FLOOD_CONST_Y = 70;

const int AMOUNT = 8;

int used[IMG_SZ_HEIGHT][IMG_SZ_WIDTH];
cv::Size sz(IMG_SZ_WIDTH / AMOUNT, IMG_SZ_HEIGHT / AMOUNT);

cv::Scalar clrWhite(255, 255, 255);
cv::Scalar clrBlack(0, 0, 0);

#endif // CONSTHEADER

