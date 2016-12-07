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
 * This interface to LSL is enspired by the LabRecorder, which is a part of 
 * the LabStreamingLayer repository.
 *
 * MUGS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MUGS.  If not, see <http://www.gnu.org/licenses/>. 
 */

#ifndef LSLINTERFACE_H
#define LSLINTERFACE_H

#include <fstream>
#include <map>
#include <list>
#include <iostream>
#include <set>
#include <vector>

#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bind.hpp>
#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/algorithm/string.hpp>

#include "liblsl/lsl_cpp.h"

#include <boost/type_traits/is_integral.hpp>
#include <boost/type_traits/is_signed.hpp>
#include <boost/type_traits/is_arithmetic.hpp>
#include <boost/type_traits/is_floating_point.hpp>

// support for endianness and binary floating-point storage
// this import scheme is part of the portable_archive code by
// christian.pfligersdorffer@eos.info (under boost license)
#if BOOST_VERSION < 103600
#include <boost/integer/endian.hpp>
#include <boost/math/fpclassify.hpp>
#elif BOOST_VERSION < 104800
#include <boost/spirit/home/support/detail/integer/endian.hpp>
#include <boost/spirit/home/support/detail/math/fpclassify.hpp>
#else
#include <boost/spirit/home/support/detail/endian/endian.hpp>
#include <boost/spirit/home/support/detail/math/fpclassify.hpp>
#endif

// namespace alias fp_classify
#if BOOST_VERSION < 103800
namespace fp = boost::math;
#else
namespace fp = boost::spirit::math;
#endif

// namespace alias endian
#if BOOST_VERSION < 104800
namespace endian = boost::detail;
#else
namespace endian = boost::spirit::detail;
#endif

using namespace lsl;

namespace detailLsl{
    // === WRITER FUNCTIONS ===
	
	/**
	 * \brief Write an integer value in little endian
 
	 *        Derived from portable archive code by christian.pfligersdorffer@eos.info (under boost license)
	 * \param[out] dst Streambuffer 
	 * \param[in] t Input value that will be written.
	 */
	template <typename T> typename boost::enable_if<boost::is_integral<T> >::type write_little_endian(std::streambuf *dst,const T &t) 
	{
	    T temp;
	    endian::store_little_endian<T,sizeof(T)>(&temp,t);
	    dst->sputn((char*)&temp,sizeof(temp));
	}
	
	/**
	 * \brief Write a floating-point value in little endian.
	 *        Derived from portable archive code by christian.pfligersdorffer@eos.info (under boost license).
	 * \param[out] dst Streambuffer
	 * \param[in] t Input value that will be written.
	 */
	template <typename T> typename boost::enable_if<boost::is_floating_point<T> >::type write_little_endian(std::streambuf *dst,const T &t) 
	{
	    typedef typename fp::detail::fp_traits<T>::type traits;
	    typename traits::bits bits;
	    // remap to bit representation
	    switch (fp::fpclassify(t)) {
		case FP_NAN: bits = traits::exponent | traits::mantissa; break;
		case FP_INFINITE: bits = traits::exponent | (t<0) * traits::sign; break;
		case FP_SUBNORMAL: assert(std::numeric_limits<T>::has_denorm);
		case FP_ZERO: // note that floats can be ±0.0
		case FP_NORMAL: traits::get_bits(t, bits); break;
		default: bits = 0; break;
	    }
	    write_little_endian(dst,bits);
	}
	
	/**
	 * \brief Write a variable-length integer (int8, int32, or int64).
	 * \param[out] dst Streambuffer
	 * \param[in] val Input value that will be written.
	 */
	inline void write_varlen_int(std::streambuf *dst, boost::uint64_t val) 
	{
	    if (val < 256) {
		dst->sputc(1);
		dst->sputc((boost::uint8_t)val);
	    } else
	    if (val <= 4294967295) {
		dst->sputc(4);
		write_little_endian(dst,(boost::uint32_t)val);
	    } else {
		dst->sputc(8);
		write_little_endian(dst,(boost::uint64_t)val);
	    }
	}
	
	/** 
	 * \brief Store a sample's values to a stream (numeric version).
	 * \param[out] dst Streambuffer
	 * \param[in] sample Sample's value that will be stored. 
	 */
	template<class T> inline void write_sample_values(std::streambuf *dst, const std::vector<T> &sample) 
	{
	    // [Value1] .. [ValueN] */
	    for (std::size_t c=0;c<sample.size();c++)
		write_little_endian(dst,sample[c]);
	}

	/** 
	 * \brief Store a sample's values to a stream (string version).
	 * \param[out] dst Streambuffer
	 * \param[in] sample Samples's value that will be stored.
	 */
	template<> inline void write_sample_values(std::streambuf *dst, const std::vector<std::string> &sample) 
	{
	    // [Value1] .. [ValueN] */
	    for (std::size_t c=0;c<sample.size();c++) {
		// [NumLengthBytes], [Length] (as varlen int)
		write_varlen_int(dst,sample[c].size());
		// [StringContent] */
		dst->sputn(sample[c].data(),sample[c].size());
	    }
	}  
}

namespace mug
{
 
    class Sample;
    
    /**
     * \brief Enumeration type that holds  the current defined chunk types.
     */
    enum ChunkTagType {
	CT_FILEHEADER = 1,      // FileHeader chunk
	CT_STREAMHEADER = 2,    // StreamHeader chunk
	CT_SAMPLES = 3,         // Samples chunk
	CT_CLOCKOFFSET = 4,     // ClockOffset chunk
	CT_BOUNDARY = 5,        // Boundary chunk
	CT_STREAMFOOTER = 6,    // StreamFooter chunk
        CT_UNDEFINED = 0
    }; 
    
    /**
     * \brief Pointer to a thread.
     */
    typedef boost::shared_ptr<boost::thread> thread_p;
    /**
     * \brief Pointer to a stream inlet.
     */
    typedef boost::shared_ptr<lsl::stream_inlet> inlet_p;
    /**
     * \brief A list of clock offset estimates (time,value).
     */
    typedef std::list<std::pair<double,double> > offset_list;
    /**
     * \brief A map from streamid to offset_list.
     */
    typedef std::map<int,offset_list> offset_lists;
    
    /**
     * \brief Interface class to collect data from LSL. 
     * \author Jonas Ditz
     */
    class LslInterface
    {
    public:
        std::string headStreamName;                /// Name of the stream that provides head coordinates
	std::string eyeStreamName;                 /// Name of the stream inlet that provides eye coordinates
	std::string stimStreamName;                /// Name of the stream inlet that provides stimulus coordinates
	int h_x, h_y, h_z, h_yaw, h_pitch, h_roll; /// Channel number of head coordinates.
	int eLeft_x, eLeft_y, eRight_x, eRight_y;  /// Channel number of eye coordinates.
	int stim_x, stim_y;                        /// Channel number of stimulus coordinates
	int terminal;                              /// Stimulus position that indicates the end of the stimulus sequence.
	
	const float boundary_interval; /// approx. interval between boundary chunks, in seconds
	const float offset_interval;    /// approx. interval between offset measurements, in seconds
	const float resolve_interval;   /// approx. interval between resolves for outstanding streams on the watchlist, in seconds
	const float chunk_interval;   /// approx. interval between resolves for outstanding streams on the watchlist, in seconds
	const float max_headers_wait;  /// maximum waiting time for moving past the headers phase while recording, in seconds
	const float max_footers_wait;   /// maximum waiting time for moving into the footers phase while recording, in seconds
	const float max_open_wait;      /// maximum waiting time for subscribing to a stream, in seconds (if exceeded, stream subscription will take place later)
	const float max_join_wait;      /// maximum time that we wait to join a thread, in seconds
	
	// the signature of the boundary chunk (next chunk begins right after this)
	const unsigned char boundary_uuid[16] = {0x43,0xA5,0x46,0xDC,0xCB,0xF5,0x41,0x0F,0xB3,0x0E,0xD5,0x46,0x73,0x83,0xCB,0xE4};
        
        /**
	 * \brief Constructor of the LslInterface class. Needs names of LSL streams for 
	 * head, eye and stimulus coordinates. The user can also specify which channel 
	 * of the LSL streams corresponds to which coordinate. The channel numbers 
	 * can be specified using arrays of the form [x, y, z, yaw, pitch, roll] for 
	 * the head LSL stream channels, [left eye x, left eye y, right eye x, right eye y] 
	 * for the eye LSL stream channels and [x, y] for the stimulus LSL stream channels.
	 * \param[in] hName Name of the stream that provides head coordinates.
	 * \param[in] eName Name of the stream that provides eye coordinates.
	 * \param[in] sName Name of the stream that provides stimulus coordinates. If no stimulus 
	 *                  is used in the experiment use an empty string.
	 * \param[in] headChannels Array that stores the channel numbers for the head coordinates.
	 * \param[in] eyeChannels Array that stores the channel numbers for the eye coordinates.
	 * \param[in] stimChannels Array that stores the channel numbers for the stimlus coordinates.
	 */
        LslInterface(std::string hName, 
		     std::string eName, 
		     std::string sName,
	             int headChannels[6], 
		     int eyeChannels[4],
		     int stimChannels[2])
	    : boundary_interval(10), offset_interval(5), resolve_interval(5), chunk_interval(0.5),
	      max_headers_wait(10), max_footers_wait(2), max_open_wait(5), max_join_wait(5),
	      unsorted_(false), shutdown_(false), streamid_(0), streaming_to_finish_(0), 
	      headers_to_finish_(0), offsets_enabled_(true)
	{
	    this->headStreamName = hName;
	    this->eyeStreamName = eName;
	    this->stimStreamName = sName;
	    this->h_x = headChannels[0]; this->h_y = headChannels[1]; this->h_z = headChannels[2];
	    this->h_yaw = headChannels[3]; this->h_pitch = headChannels[4]; this->h_roll = headChannels[5];
	    this->eLeft_x = eyeChannels[0]; this->eLeft_y = eyeChannels[1];
	    this->eRight_x = eyeChannels[2]; this->eRight_y = eyeChannels[3];
	    this->stim_x = stimChannels[0]; this->stim_y = stimChannels[1];
	}
	
	/**
	 * \brief Constructor of the LslInterface class. Needs names of LSL streams for 
	 * head, eye and stimulus coordinates. This constructor assigns default 
	 * values to the LSL stream channel numbers. The channel numbers are:
	 *   x coordinate of head           at channel 0   of head LSL stream
	 *   y coordinate of head           at channel 1   of head LSL stream
	 *   z coordinate of head           at channel 2   of head LSL stream
	 *   yaw coordinate of head         at channel 3   of head LSL stream
	 *   pitch coordinate of head       at channel 4   of head LSL stream
	 *   roll coordinate of head        at channel 5   of head LSL stream
	 * 
	 *   x coordinate of the left eye   at channel 0   of eye LSL stream
	 *   y coordinate of the left eye   at channel 1   of eye LSL stream
	 *   x coordinate of the right eye  at channel 2   of eye LSL stream
	 *   y coordinate of the right eye  at channel 3   of eye LSL stream
	 * 
	 *   x coordinate of stimulus       at channel 0   of stimulus LSL stream
	 *   y coordinate of stimulus       at channel 1   of stimulus LSL stream
	 * \param[in] hName Name of the stream that provides head coordinates.
	 * \param[in] eName Name of the stream that provides eye coordinates.
	 * \param[in] sName Name of the stream that provides stimulus coordinates. If no stimulus 
	 *                  is used in the experiment use an empty string.
	 */
	LslInterface(std::string hName, 
		     std::string eName, 
		     std::string sName)
	    : boundary_interval(10), offset_interval(5), resolve_interval(5), chunk_interval(0.5),
	      max_headers_wait(10), max_footers_wait(2), max_open_wait(5), max_join_wait(5),
	      unsorted_(false), shutdown_(false), streamid_(0), streaming_to_finish_(0), 
	      headers_to_finish_(0), offsets_enabled_(true)
	{
            this->headStreamName = hName;
	    this->eyeStreamName = eName;
	    this->stimStreamName = sName;
	    this->h_x = 0; this->h_y = 1; this->h_z = 2;
	    this->h_yaw = 3; this->h_pitch = 4; this->h_roll = 5;
	    this->eLeft_x = 0; this->eLeft_y = 1;
	    this->eRight_x = 2; this->eRight_y = 3;
	    this->stim_x = 0; this->stim_y = 1;
	    
	    std::cout <<"shutdown_: " << shutdown_ << std::endl;
	}
	
	/**
	 * \brief Destructor.
	 */
	~LslInterface() 
	{
	    try {	      
		// set the shutdown flag (from now on no more new streams)
		{
	            boost::mutex::scoped_lock lock(phase_mut_);
		    shutdown_ = true;
		}
		// stop the Boundary writer thread
		boundary_thread_->interrupt();
		boundary_thread_->timed_join(boost::posix_time::milliseconds((boost::int64_t)(max_join_wait*1000.0)));
		// wait for all stream threads to join...
		for (std::size_t k=0;k<stream_threads_.size();k++)
		    stream_threads_[k]->timed_join(boost::posix_time::milliseconds((boost::int64_t)(max_join_wait*1000.0)));
		std::cout << "[MUGS LslInterface] Closing the file." << std::endl;
	    } 
	    catch(std::exception &e) {
	        std::cout << "[MUGS LslInterface] Error while closing the recording: " << e.what() << std::endl;
	    }
	}
        
        /**
	 * \brief fetch data of an experiment without a presented stimulus.
	 * \param[in] filename Name of the file that stores recorded data.
	 */
	void fetchData(std::string& filename);
        
        /**
	 * \brief Fetch data of an experiment with a presented stimulus.
	 * \param[in] filename Name of the file that stores recorded data.
	 * \param[in] terminal Integer value that is send by the stimulus stream 
	 *                     to indicate the end of the experiment.
	 */
	void fetchData(std::string& filename, int terminal);
	
    private:
        // the file stream
	boost::iostreams::filtering_ostream file_;  /// the file output stream
	boost::mutex chunk_mut_;
      
        // static information
	bool offsets_enabled_;  /// whether to collect time offset information alongside with the stream contents
	bool unsorted_;         /// whether this file may contain unsorted chunks (e.g., of late streams)
	
	// streamid allocation
	boost::uint32_t streamid_;     /// the highest streamid allocated so far
	boost::mutex streamid_mut_;    /// a mutex to protect the streamid
	
	// phase-of-recording state (headers, streaming data, or footers)
	bool shutdown_;                        /// whether we are trying to shut down
	boost::uint32_t headers_to_finish_;    /// the number of streams that still need to write their header (i.e., are not yet ready to write streaming content)
	boost::uint32_t streaming_to_finish_;  /// the number of streams that still need to finish the streaming phase (i.e., are not yet ready for writing their footer)
	boost::condition ready_for_streaming_; /// condition variable signaling that all streams have finished writing their headers and are now ready to write streaming content
	boost::condition ready_for_footers_;   /// condition variable signaling that all streams have finished their recording jobs and are now ready to write a footer
	boost::mutex phase_mut_;               /// a mutex to protect the phase state

	// data structure to collect the time offsets for every stream
	offset_lists offset_lists_;  /// the clock offset lists for each stream (to be written into the footer)
	boost::mutex offset_mut_;    /// a mutex to protect the offset lists

	// data for shutdown / final joining
	std::vector<thread_p> stream_threads_;  /// the spawned stream handling threads
	thread_p boundary_thread_;              /// the spawned boundary-recording thread
	
	/** 
	 * \brief Record from a given stream (identified by its streaminfo).
	 * \param[in] src The stream_info from which to record.
	 * \param[in] phase_locked Whether this is a stream that is locked to the phases (1. Headers, 2. Streaming Content, 3. Footers).
	 *                         Late-added streams (e.g. forgotten devices) are not phase-locked.
	 */
	void record_from_streaminfo(lsl::stream_info src, bool phase_locked);
	
	/**
	 * \brief Record boundary markers every few seconds.
	 */
	void record_boundaries();
	
	/**
	 * \brief Record ClockOffset chunks from a given stream.
	 * \param[in] streamid Id of the stream for which the offset is recorded.
	 * \param[in] in Pointer to the stream inlet of the stream.
	 */
	void record_offsets(boost::uint32_t streamid, inlet_p in);
	
	/**
	 * \brief Sample collection loop for a numeric stream.
	 * \param[in] streamid Id of the stream that is used.
	 * \param[in] srate Sampling rate of the stream.
	 * \param[in] sname Name of the stream.
	 * \param[out] first_timestamp First timestamp. Used for synchronisation.
	 * \param[out] last_timestamp Last timestamp. Used for synchronisation.
	 * \param[out] sample_count Counter to track the number of samples.
	 */
	template<typename T> 
	void typed_transfer_loop(uint32_t streamid, double srate, std::string sname, inlet_p in, 
						    double& first_timestamp, double& last_timestamp, uint64_t& sample_count)
	{
	    thread_p offset_thread;
	    try {
		// optionally start an offset collection thread for this stream
		if (offsets_enabled_)
		    offset_thread.reset(new boost::thread(&LslInterface::record_offsets,this,streamid,in));

		first_timestamp = -1.0;
		last_timestamp = 0.0;
		sample_count = 0;
		double sample_interval = srate ? 1.0/srate : 0;

		// temporary data
		std::vector<std::vector<T> > chunk;
		std::vector<double> timestamps;
		while (true) {
		    //std::cout << "reading loop of stream " << streamid << ", shutdown_ = " << shutdown_ << std::endl;
		  
		    // check for shutdown condition
		    {
			boost::mutex::scoped_lock lock(phase_mut_);
			if (shutdown_)
			    break;
		    }

		    // get a chunk from the stream
		    if (in->pull_chunk(chunk,timestamps)) {
			if (first_timestamp == -1.0)
			    first_timestamp = timestamps[0];
			// generate [Samples] chunk contents...
			std::ostringstream content;
			// [StreamId]
			detailLsl::write_little_endian(content.rdbuf(),streamid); 
			// [NumSamplesBytes], [NumSamples]
			detailLsl::write_varlen_int(content.rdbuf(),chunk.size());
			// for each sample...
			for (std::size_t s=0;s<chunk.size();s++) {
			    // if the time stamp can be deduced from the previous one...
			    if (last_timestamp + sample_interval == timestamps[s]) {
				// [TimeStampBytes] (0 for no time stamp)
				content.rdbuf()->sputc(0);
			    } else {
				// [TimeStampBytes]
				content.rdbuf()->sputc(8);
				// [TimeStamp]
				detailLsl::write_little_endian(content.rdbuf(),timestamps[s]);
			    }
			    // [Sample1] .. [SampleN]
			    detailLsl::write_sample_values<T>(content.rdbuf(),chunk[s]);
			    last_timestamp = timestamps[s];
			    sample_count++;
			}
			// write the actual chunk
			write_chunk(mug::CT_SAMPLES,content.str());
		    } else 
			boost::this_thread::sleep(boost::posix_time::milliseconds((int)(chunk_interval*1000)));

		}

		// terminate the offset collection thread, too
		if (offset_thread) {
		    offset_thread->interrupt();
		    offset_thread->timed_join(boost::posix_time::milliseconds((boost::int64_t)(max_join_wait*1000.0)));
		}
	    } 
	    catch(std::exception &) {
		if (offset_thread) {
		    offset_thread->interrupt();
		    offset_thread->timed_join(boost::posix_time::milliseconds((boost::int64_t)(max_join_wait*1000.0)));
		}
		throw;
	    }
	}
	
	
	// === WRITER FUNCTIONS ===
	
	/**
	 * \brief Write a generic chunk.
	 * \param[in] tag Specifies the type of the chunk.
	 * \param[in] content Content that should be written to the chunk.
	 */
	void write_chunk(ChunkTagType tag, const std::string &content) {
	    // lock the file stream...
	    boost::mutex::scoped_lock lock(chunk_mut_);
	    // [NumLengthBytes], [Length] (variable-length integer)
	    std::size_t len = 2 + content.size();
	    detailLsl::write_varlen_int(file_.rdbuf(),len);
	    // [Tag]
	    detailLsl::write_little_endian(file_.rdbuf(),(boost::uint16_t)tag);
	    // [Content]
	    file_.rdbuf()->sputn(content.data(),content.size());
	}
	
	
	// === PHASE REGISTRATION & CONDITION CHECKS ===
	
	/**
	 * \brief Stream enters the header phase.
	 * \param[in] phase_locked Whether this stream has a phase lock or not.
	 */
	void enter_headers_phase(bool phase_locked) {
	    if (phase_locked) {
		boost::mutex::scoped_lock lock(phase_mut_);
		headers_to_finish_++;
	    }
	}

	/**
	 * \brief Stream leaves the header phase.
	 * \param[in] phase_locked Whether this stream has a phase lock or not.
	 */
	void leave_headers_phase(bool phase_locked) {
	    if (phase_locked) {
		boost::mutex::scoped_lock lock(phase_mut_);
		headers_to_finish_--;
		lock.unlock();
		ready_for_streaming_.notify_all();
	    }
	}

	/**
	 * \brief Stream enters the streaming phase.
	 * \param[in] phase_locked Whether this stream has a phase lock or not.
	 */
	void enter_streaming_phase(bool phase_locked) {
	    if (phase_locked) {
		boost::mutex::scoped_lock lock(phase_mut_);
		ready_for_streaming_.timed_wait(lock, boost::posix_time::milliseconds((boost::int64_t)(max_headers_wait*1000.0)),boost::bind(&LslInterface::ready_for_streaming,this));
		streaming_to_finish_++;
	    }
	}

	/**
	 * \brief Stream leaves the streaming phase.
	 * \param[in] phase_locked Whether this stream has a phase lock or not.
	 */
	void leave_streaming_phase(bool phase_locked) {
	    if (phase_locked) {
		boost::mutex::scoped_lock lock(phase_mut_);
		streaming_to_finish_--;
		lock.unlock();
		ready_for_footers_.notify_all();
	    }
	}

	/**
	 * \brief Stream enters the footer phase.
	 * \param[in] phase_locked Whether this stream has a phase lock or not.
	 */
	void enter_footers_phase(bool phase_locked) {
	    if (phase_locked) {
		boost::mutex::scoped_lock lock(phase_mut_);
		ready_for_footers_.timed_wait(lock, boost::posix_time::milliseconds((boost::int64_t)(max_footers_wait*1000.0)),boost::bind(&LslInterface::ready_for_footers,this));
	    }
	}

	/**
	 * \brief Stream leaves the footer phase.
	 * \param[in] phase_locked Whether this stream has a phase lock or not.
	 */
	void leave_footers_phase(bool phase_locked) { /* noting to do */ }
	
	/**
	 * \brief A condition that indicates that we're ready to write streaming content into the file.
	 */
	bool ready_for_streaming() const { return headers_to_finish_ <= 0; }
	
	/**
	 * \brief A condition that indicates that we're ready to write footers into the file.
	 */
	bool ready_for_footers() const { return streaming_to_finish_ <= 0 && headers_to_finish_ <= 0; }
	
	/**
	 * \brief Allocate a fresh stream id.
	 */
	boost::uint32_t fresh_streamid() {
		boost::mutex::scoped_lock lock(streamid_mut_);
		return ++streamid_;
	}
    };
    
    
    template<> inline
    void LslInterface::typed_transfer_loop<boost::int16_t>(uint32_t streamid, double srate, std::string sname, inlet_p in, 
								double& first_timestamp, double& last_timestamp, uint64_t& sample_count)
    {
	thread_p offset_thread;
	try {
	    // optionally start an offset collection thread for this stream
	    if (offsets_enabled_)
		offset_thread.reset(new boost::thread(&LslInterface::record_offsets,this,streamid,in));

	    first_timestamp = -1.0;
	    last_timestamp = 0.0;
	    sample_count = 0;
	    double sample_interval = srate ? 1.0/srate : 0;

	    // temporary data
	    std::vector<std::vector<boost::int16_t> > chunk;
	    std::vector<double> timestamps;
	    while (true) {
		//std::cout << "reading loop of stream " << streamid << ", shutdown_ = " << shutdown_ << std::endl;
	      
		// check for shutdown condition
		{
		    boost::mutex::scoped_lock lock(phase_mut_);
		    if (shutdown_)
			break;
		}

		// get a chunk from the stream
		if (in->pull_chunk(chunk,timestamps)) {
		    if (first_timestamp == -1.0)
			first_timestamp = timestamps[0];
		    // generate [Samples] chunk contents...
		    std::ostringstream content;
		    // [StreamId]
		    detailLsl::write_little_endian(content.rdbuf(),streamid); 
		    // [NumSamplesBytes], [NumSamples]
		    detailLsl::write_varlen_int(content.rdbuf(),chunk.size());
		    // for each sample...
		    for (std::size_t s=0;s<chunk.size();s++) {
			// if the time stamp can be deduced from the previous one...
			if (last_timestamp + sample_interval == timestamps[s]) {
			    // [TimeStampBytes] (0 for no time stamp)
			    content.rdbuf()->sputc(0);
			} else {
			    // [TimeStampBytes]
			    content.rdbuf()->sputc(8);
			    // [TimeStamp]
			    detailLsl::write_little_endian(content.rdbuf(),timestamps[s]);
			}
			// [Sample1] .. [SampleN]
			if (sname == stimStreamName && chunk[s][0] == terminal){
			    shutdown_ = true;
			    break;
			}
			detailLsl::write_sample_values<boost::int16_t>(content.rdbuf(),chunk[s]);
			last_timestamp = timestamps[s];
			sample_count++;
		    }
		    // write the actual chunk
		    write_chunk(mug::CT_SAMPLES,content.str());
		} else 
		    boost::this_thread::sleep(boost::posix_time::milliseconds((int)(chunk_interval*1000)));

	    }

	    // terminate the offset collection thread, too
	    if (offset_thread) {
		offset_thread->interrupt();
		offset_thread->timed_join(boost::posix_time::milliseconds((boost::int64_t)(max_join_wait*1000.0)));
	    }
	} 
	catch(std::exception &) {
	    if (offset_thread) {
		offset_thread->interrupt();
		offset_thread->timed_join(boost::posix_time::milliseconds((boost::int64_t)(max_join_wait*1000.0)));
	    }
	    throw;
	}
    }
}

#endif
