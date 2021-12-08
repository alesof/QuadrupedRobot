#include "QrToPng.h"

int main() {

    std::string ID;
    std::string name;

    std::cout<<"Inserisci ID per creare QRCode: \n";
    std::getline (std::cin,ID);

    std::cout<<"Inserire nome file .svg: \n";
	std::getline(std::cin,name); //Nessun check su inserito o meno .svg (?)



    int imgSize = 300;
    int minModulePixelSize = 3;
    auto PngOutput = QrToPng(name, imgSize, minModulePixelSize, ID, true, qrcodegen::QrCode::Ecc::MEDIUM);

    PngOutput.writeToPNG();
    return 0;
}