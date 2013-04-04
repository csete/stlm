#include <Ice/Ice.h>
#include "gnuradio.h"

using namespace std;
using namespace GNURadio;

int main(int argc, char* argv[])
{
    int status = 0;
    Ice::CommunicatorPtr ic;

    GNURadio::KnobIDList   empty_list;  // vector<string>
    GNURadio::KnobPropMap  knob_props;  // map<string, GNURadio::KnobProp>
    GNURadio::KnobMap      knob_map;    // map<string, GNURadio::KnobPtr>
    GNURadio::KnobPtr      knob;
    GNURadio::KnobIPtr     knobi;
    GNURadio::KnobDPtr     knobd;

    try
    {
        // Get proxy object
        ic = Ice::initialize(argc, argv);
        Ice::ObjectPrx base = ic->stringToProxy("gnuradio:tcp -h localhost -p 43243");
        ControlPortPrx ctrlport = ControlPortPrx::checkedCast(base);

        if (!ctrlport)
            throw "Invalid proxy";

        // Get list of knob properties
        knob_props = ctrlport->properties(empty_list);
        //for (KnobPropMap::iterator it = knob_props.begin(); it != knob_props.end(); ++it)
        //    cout << "  " << it->first << endl;

        // Get list of knobs
        knob_map = ctrlport->get(empty_list);
        cout << "Exported knobs: " << knob_map.size() << endl;

        for (KnobMap::iterator it = knob_map.begin(); it != knob_map.end(); ++it)
        {
            cout << "  " << it->first << " = ";

            knob = it->second;

            switch (knob_props[it->first].type)
            {
            case KNOBINT:
                knobi = static_cast<KnobIPtr>(knob);
                cout << knobi->value << endl;
                break;

            case KNOBDOUBLE:
                knobd = static_cast<KnobDPtr>(knob); // can also use: KnobPtr::dynamicCast(knob);
                cout << knobd->value << endl;
                break;

            default:
                cout << "(unsupported type)" << endl;
            }
        }

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

