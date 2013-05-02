/*
 * Copyright (C) 2013 Alexandru Csete, OZ9AEC
 *
 * Strx is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * Strx is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "receiver.h"

// other includes
#include <boost/program_options.hpp>
#include <iostream>
#include <string>

namespace po = boost::program_options;

/*! Convert command line argument to frequency in Hz.
 * \param arg The command line argument, e.g. 2400M
 * \return The frequency in Hz.
 */
double arg_to_freq(const std::string arg)
{
    std::string _freq = arg;
    double freq;
    double suff = 1.0;

	switch (_freq[_freq.length()-1])
    {
        case 'G':
            suff = 1.e9;
            _freq.erase(_freq.end()-1, _freq.end());
            break;
		case 'M':
			suff = 1.e6;
            _freq.erase(_freq.end()-1, _freq.end());
            break;
		case 'k':
			suff = 1.e3;
            _freq.erase(_freq.end()-1, _freq.end());
            break;
    }

    freq = atof(_freq.c_str()) * suff;

    return freq;
}

int main(int argc, char **argv)
{
    receiver *rx;

    // command line options
    std::string freq_str;
    std::string ant_str;
    double freq;
    double gain;
    bool clierr=false;
    std::string rxname;
    std::string input;
    std::string output;

    po::options_description desc("Command line options");
    desc.add_options()
        ("help,h", "This help message")
        ("name,n", po::value<std::string>(&rxname)->default_value(""), "Receiver name (used for ctrlport)")
        ("input,i", po::value<std::string>(&input)->default_value(""), "USRP sub device or I/Q file (use file:/path/to/file)")
        ("ant,a", po::value<std::string>(&ant_str), "Select USRP antenna (e.g. RX2)")
        ("freq,f", po::value<std::string>(&freq_str), "RF frequency in Hz or using G, M, k suffix")
        ("gain,g", po::value<double>(&gain), "RF/IF gain in dB")
        ("output,o", po::value<std::string>(&output)->default_value(""), "Output file (use stdout if omitted)")
    ;
    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    }
    catch(const boost::program_options::invalid_command_line_syntax& ex)
    {
        clierr = true;
    }
    po::notify(vm);

    if (vm.count("help") || clierr)
    {
        std::cout << "Sapphire telemetry receiver " << VERSION << std::endl << desc << std::endl;
        return 1;
    }

    // create receiver and set paarameters
    rx = new receiver(rxname, input, output);

    if (vm.count("freq"))
    {
        freq = arg_to_freq(freq_str);
        rx->set_rf_freq(freq);
    }
    if (vm.count("gain"))
    {
        rx->set_rf_gain(gain);
    }
    if (vm.count("ant"))
    {
        rx->set_antenna(ant_str);
    }

    rx->start();

    delete rx;

    return 0;
}
