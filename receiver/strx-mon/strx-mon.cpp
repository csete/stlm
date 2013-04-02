#include <Ice/Ice.h>
#include <gnuradio.h>

using namespace std;
using namespace GNURadio;

int main(int argc, char* argv[])
{
    int status = 0;
    Ice::CommunicatorPtr ic;

    GNURadio::KnobIDList   empty_list;  // vector<string>
    GNURadio::KnobPropMap  knob_props;  // map<string, GNURadio::KnobProp>

    try
    {
        // Get proxy object
        ic = Ice::initialize(argc, argv);
        Ice::ObjectPrx base = ic->stringToProxy("gnuradio:tcp -h localhost -p 43243");
        ControlPortPrx ctrlport = ControlPortPrx::checkedCast(base);

        if (!ctrlport)
            throw "Invalid proxy";

        // Get list of knobs
        knob_props = ctrlport->properties(empty_list);
        cout << "Exported knobs: " << knob_props.size() << endl;

        for (KnobPropMap::iterator it = knob_props.begin(); it != knob_props.end(); ++it)
            cout << "  " << it->first << endl;

    }
    catch (const Ice::Exception& ex)
    {
        cerr << ex << endl;
        status = 1;
    }
    catch (const char* msg)
    {
        cerr << msg << endl;
        status = 1;
    }

    if (ic)
        ic->destroy();

    return status;
}

