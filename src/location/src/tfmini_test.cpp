#include <TFplus.h>
#include <tf_mini_i2c.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    benewake::TFplus *tfplus_obj;
    TFMiniPro *tf_mini_plus_;

    if (argc == 1)
    {
        std::cout << "i2c or uart ?" << std::endl;
        return 0;
    }
    else if (argc == 2)
    {
        if (strcmp(argv[1], "uart") == 0)
        {
            std::cout << "Open Serial port: /dev/ttyTHS0" << std::endl;
            tfplus_obj = new benewake::TFplus("/dev/ttyTHS0", 115200);
        }
        else if (strcmp(argv[1], "i2c") == 0)
        {
            try
            {
                tf_mini_plus_ = new TFMiniPro(0, 1);
                std::cout << "Range sensor ready on i2c bus: 1, address: 0x00." << std::endl;
            }
            catch (const std::system_error &error)
            {
                std::cout << "ERROR opening sensor on on i2c bus: 1, address: 0x00." << std::endl;
                return 0;
            }
        }
    }
    else if (argc == 3)
    {
        if (strcmp(argv[1], "uart") == 0)
        {
            std::cout << "Open Serial port:" << argv[2] << std::endl;
            tfplus_obj = new benewake::TFplus(argv[2], 115200);
        }
        else if (strcmp(argv[1], "i2c"))
        {
            std::cout << "Lacking params." << std::endl;
        }
    }
    else if (argc == 4)
    {

        if (strcmp(argv[1], "i2c") == 0)
        {
            try
            {
                tf_mini_plus_ = new TFMiniPro(std::stoi(argv[3]), std::stoi(argv[2]));
                std::cout << "Range sensor ready on i2c bus: " << std::stoi(argv[2]) << ", address: 0x" << std::hex << std::stoi(argv[3]) << std::endl;
            }
            catch (const std::system_error &error)
            {
                std::cout << "ERROR opening sensor on i2c bus: " << std::stoi(argv[2]) << ", address: 0x" << std::hex << std::stoi(argv[3]) << std::endl;
                return 0;
            }
        }
    }
    std::cout << "tf mini init" << std::endl;

    float dist;
    while (true)
    {

        if (argc > 1)
        {
            if (strcmp(argv[1], "uart") == 0)
            {
                dist = tfplus_obj->getDist();
                std::cout << "uart: dist:" << dist << std::endl;
            }
            else if (strcmp(argv[1], "i2c") == 0)
            {
                dist = tf_mini_plus_->ReadData();
                std::cout << "i2c: dist:" << dist << std::endl;
                usleep(1000 * 10);
            }
        }
    }
    if (strcmp(argv[1], "uart") == 0)
    {
        tfplus_obj->closePort();
    }
    // else if (argv[1] == "i2c")
    // {
    //     dist = tf_mini_plus_->ReadData();
    // }

    return 0;
}