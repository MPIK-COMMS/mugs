/* 
 * Copyright (c) 2013, 2016 Max Planck Institute for Biological Cybernetics
 * All rights reserved.
 * 
 * This file is part of MUGS - Mobile and Unrestrained Gazetracking Software.
 *
 * MUGS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MUGS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MUGS.  If not, see <http://www.gnu.org/licenses/>. 
 */

#include "liblsl/lsl_cpp.h"
#include <stdlib.h>
#include <stdio.h>
#include <ctime>
#include <vector>

#include "mug/lsl_interface.h"
#include "mug/sample.h"

using namespace mug;
using namespace detailLsl;

void LslInterface::fetchData(std::string& filename )
{
    // resolve head stream
    std::cout << "[MUGS LslInterface] Resolving head tracker stream..." << std::endl;
    std::vector<lsl::stream_info> resultsHead = lsl::resolve_stream("name", this->headStreamName);
    std::cout << "  DONE\n" << std::endl;
    // resolve eye stream
    std::cout << "[MUGS LslInterface] Resolving eye tracker stream..." << std::endl;
    std::vector<lsl::stream_info> resultsEye = lsl::resolve_stream("name", this->eyeStreamName);
    std::cout << "  DONE\n" << std::endl;
    
    std::vector<lsl::stream_info> streams;
    streams.push_back(resultsHead[0]);
    streams.push_back(resultsEye[0]);
        
    // open file stream
    std::cout << "[MUGS LslInterface] Opening file " << filename << " ... ";
    if (boost::iends_with(filename,".xdfz"))
        file_.push(boost::iostreams::zlib_compressor());
    file_.push(boost::iostreams::file_descriptor_sink(filename,std::ios::binary | std::ios::trunc));
    std::cout << "done." << std::endl;
    // [MagicCode]
    file_.rdbuf()->sputn("XDF:",4);
    // [FileHeader] chunk
    write_chunk(mug::CT_FILEHEADER,"<?xml version=\"1.0\"?><info><version>1.0</version></info>");
    // create a recording thread for each stream
    for (std::size_t k=0;k<streams.size();k++)
        stream_threads_.push_back(thread_p(new boost::thread(&LslInterface::record_from_streaminfo,this,streams[k],true)));
    // create a boundary chunk writer thread
    boundary_thread_.reset(new boost::thread(&LslInterface::record_boundaries,this));
    
    // wait until user ends the recording
    std::cout << "\n[MUGS LslInterface] Recording tracking data. Press ENTER to stop...\n" << std::endl;
    int c;
    c = std::getchar();
}


void LslInterface::fetchData(std::string& filename , int terminal)
{
    // resolve head stream
    std::cout << "[MUGS LslInterface] Resolving head tracker stream..." << std::endl;
    std::vector<lsl::stream_info> resultsHead = lsl::resolve_stream("name", this->headStreamName);
    std::cout << "  DONE\n" << std::endl;
    // resolve eye stream
    std::cout << "[MUGS LslInterface] Resolving eye tracker stream..." << std::endl;
    std::vector<lsl::stream_info> resultsEye = lsl::resolve_stream("name", this->eyeStreamName);
    std::cout << "  DONE\n" << std::endl;
    // resolve stimulus stream
    std::cout << "[MUGS LslInterface] Resolving stimulus stream..." << std::endl;
    std::vector<lsl::stream_info> resultsStim = lsl::resolve_stream("name", this->stimStreamName);
    std::cout << "  DONE\n" << std::endl;
    
    std::vector<lsl::stream_info> streams;
    streams.push_back(resultsHead[0]);
    streams.push_back(resultsEye[0]);
    streams.push_back(resultsStim[0]);
    this->terminal = terminal;
    
    // open file stream
    std::cout << "[MUGS LslInterface] Opening file " << filename << " ... ";
    if (boost::iends_with(filename,".xdfz"))
        file_.push(boost::iostreams::zlib_compressor());
    file_.push(boost::iostreams::file_descriptor_sink(filename,std::ios::binary | std::ios::trunc));
    std::cout << "done." << std::endl;
    // [MagicCode]
    file_.rdbuf()->sputn("XDF:",4);
    // [FileHeader] chunk
    write_chunk(mug::CT_FILEHEADER,"<?xml version=\"1.0\"?><info><version>1.0</version></info>");
    // create a recording thread for each stream
    for (std::size_t k=0;k<streams.size();k++)
        stream_threads_.push_back(thread_p(new boost::thread(&LslInterface::record_from_streaminfo,this,streams[k],true)));
    // create a boundary chunk writer thread
    boundary_thread_.reset(new boost::thread(&LslInterface::record_boundaries,this));
    
    // wait until the stimulus representation finished
    while(!shutdown_) {
        boost::this_thread::sleep(boost::posix_time::millisec(10000));
    }
}


void LslInterface::record_from_streaminfo(lsl::stream_info src, bool phase_locked)
{
    try {
        double first_timestamp, last_timestamp;
	boost::uint64_t sample_count;
	// obtain a fresh streamid
	boost::uint32_t streamid = fresh_streamid();

	inlet_p in;
	lsl::stream_info info;
	
	// --- headers phase
	try {
	    enter_headers_phase(phase_locked);

	    // open an inlet to read from (and subscribe to data immediately)
	    in.reset(new lsl::stream_inlet(src));
	    try {
		in->open_stream(max_open_wait);
		std::cout << "[MUGS LslInterface] Opened the stream " << src.name() << "." << std::endl;
	    } catch(lsl::timeout_error &) {
		std::cout << "[MUGS LslInterface] Subscribing to the stream " << src.name() << " is taking relatively long; collection from this stream will be delayed." << std::endl;
	    }

	    // retrieve the stream header & get its XML version
	    info = in->info();
	    std::string as_xml = info.as_xml();
	    // generate the [StreamHeader] chunk contents...
	    std::ostringstream hdr_content;
	    // [StreamId]
	    write_little_endian(hdr_content.rdbuf(),streamid); 
	    // [Content]
	    hdr_content.rdbuf()->sputn(&as_xml[0],as_xml.size());
	    // write the actual chunk
	    write_chunk(CT_STREAMHEADER,hdr_content.str());
	    std::cout << "[MUGS LslInterface] Received header for stream " << src.name() << "." << std::endl;
	    
	    leave_headers_phase(phase_locked);
	} catch(std::exception &) { 
	    leave_headers_phase(phase_locked);
	    throw;
	}

	// --- streaming phase
	try {
	    // this waits until we are done writing all headers for the initial set of (phase-locked) streams (any streams that are discovered later, if any, will not wait)
	    // we're doing this so that all headers of the initial set of streams come first, so the XDF file is properly sorted unless we discover some streams later which 
	    // someone "forgot to turn on" before the recording started; in that case the file would have to be post-processed to be in properly sorted (seekable) format
	    enter_streaming_phase(phase_locked);
	    std::cout << "[MUGS LslInterface] Started data collection for stream " << src.name() << "." << std::endl;
	    
	    // now write the actual sample chunks...
	    switch (src.channel_format()) {
	        case lsl::cf_int8:
		    typed_transfer_loop<char>(streamid,info.nominal_srate(),src.name(),in,first_timestamp,last_timestamp,sample_count);
		    break;
		case lsl::cf_int16:
		    typed_transfer_loop<boost::int16_t>(streamid,info.nominal_srate(),src.name(),in,first_timestamp,last_timestamp,sample_count);
		    break;
		case lsl::cf_int32:
		    typed_transfer_loop<boost::int32_t>(streamid,info.nominal_srate(),src.name(),in,first_timestamp,last_timestamp,sample_count);
		    break;
		case lsl::cf_float32:
		    typed_transfer_loop<float>(streamid,info.nominal_srate(),src.name(),in,first_timestamp,last_timestamp,sample_count);
		    break;
		case lsl::cf_double64:
		    typed_transfer_loop<double>(streamid,info.nominal_srate(),src.name(),in,first_timestamp,last_timestamp,sample_count);
		    break;
		case lsl::cf_string:
		    typed_transfer_loop<std::string>(streamid,info.nominal_srate(),src.name(),in,first_timestamp,last_timestamp,sample_count);
		    break;
		default:
		    // unsupported channel format
		    throw std::runtime_error(std::string("[MUGS LslInterface] Unsupported channel format in stream ") += src.name());
		}

	    leave_streaming_phase(phase_locked);
	} 
	catch(std::exception &) {
	    leave_streaming_phase(phase_locked);
	    throw;
	}

	// --- footers phase
	try {
	    enter_footers_phase(phase_locked);

	    // now generate the [StreamFooter] contents
	    std::ostringstream footer; footer.precision(16); 
	    // [StreamId]
	    write_little_endian(footer.rdbuf(),streamid); 
	    // [Content]
	    footer << "<?xml version=\"1.0\"?><info><first_timestamp>" << first_timestamp << "</first_timestamp><last_timestamp>" << last_timestamp << "</last_timestamp><sample_count>" << sample_count << "</sample_count>";
	    footer << "<clock_offsets>";
	    {
	        // including the clock_offset list
		boost::mutex::scoped_lock lock(offset_mut_);
		for (offset_list::iterator i=offset_lists_[streamid].begin(),e=offset_lists_[streamid].end();i!=e;i++) {
		    footer << "<offset><time>" << i->first << "</time><value>" << i->second << "</value></offset>";
		}
		footer << "</clock_offsets></info>";
		write_chunk(CT_STREAMFOOTER,footer.str());
	    }

	    std::cout << "[MUGS LslInterface] Wrote footer for stream " << src.name() << "." << std::endl;
	    leave_footers_phase(phase_locked);
	} 
	catch(std::exception &) {
	    leave_footers_phase(phase_locked);
	    throw;
	}
    } 
    catch(boost::thread_interrupted &) { 
    }
    catch(std::exception &e) {
        std::cout << "[MUGS LslInterface] Error in the record_from_streaminfo thread: " << e.what() << std::endl;
    }
}


void LslInterface::record_boundaries()
{
    try {
	while (!shutdown_) {
	    // sleep for the interval
	    boost::this_thread::sleep(boost::posix_time::milliseconds((int)(boundary_interval*1000)));
	    // write a [Boundary] chunk...
	    write_chunk(mug::CT_BOUNDARY, std::string((char*)&boundary_uuid[0],16));
	}
    } 
    catch(boost::thread_interrupted &) { 
    }
    catch(std::exception &e) {
        std::cout << "[MUGS LslInterface] Error in the record_boundaries thread: " << e.what() << std::endl;
    }
}


void LslInterface::record_offsets(uint32_t streamid, inlet_p in)
{
    try {
	while (!shutdown_) {
	    // sleep for the interval
	    boost::this_thread::sleep(boost::posix_time::milliseconds((int)(offset_interval*1000)));
	    // query the time offset
	    double offset, now;
	    try {
	        offset = in->time_correction(2);
		now = lsl::local_clock();
	    }
	    catch (lsl::timeout_error &) { continue; }
	    // generate the [ClockOffset] chunk contents
	    std::ostringstream content; 
	    // [StreamId]
	    write_little_endian(content.rdbuf(),streamid); 
	    // [CollectionTime]
	    write_little_endian(content.rdbuf(),now-offset);
	    // [OffsetValue]
	    write_little_endian(content.rdbuf(),offset);
	    // write the chunk
	    write_chunk(mug::CT_CLOCKOFFSET,content.str());
	    // also append to the offset lists
	    boost::mutex::scoped_lock lock(offset_mut_);
	    offset_lists_[streamid].push_back(std::make_pair(now-offset,offset));
	}
    } 
    catch(boost::thread_interrupted &) { 
    }
    catch(std::exception &e) {
	std::cout << "[MUGS LslInterface] Error in the record_offsets thread: " << e.what() << std::endl;
    }
}


