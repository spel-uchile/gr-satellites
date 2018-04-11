#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: SUCHAI 1 Decoder
# Author: Carlos Gonzalez
# Description: SUCHAI 1 Decoder based on Daniel Estevez's GOMX-1 decoder
# Generated: Wed Apr 11 19:33:24 2018
##################################################

from gnuradio import analog
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio import zeromq
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import numpy
import satellites


class suchai(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "SUCHAI 1 Decoder")

        ##################################################
        # Variables
        ##################################################
        self.threshold = threshold = 2
        self.sync = sync = "11000011101010100110011001010101"
        self.samples_symbol = samples_symbol = 20
        self.samp_rate = samp_rate = 48000
        self.packlen = packlen = (255+3)*8
        self.demod_freq = demod_freq = -1800
        self.cutoff_freq = cutoff_freq = 1400

        ##################################################
        # Blocks
        ##################################################
        self.zeromq_pub_msg_sink_0 = zeromq.pub_msg_sink('tcp://127.0.0.1:5556', 100)
        self.satellites_u482c_decode_0 = satellites.u482c_decode(False, 0, 1, 1)
        self.satellites_suchai_telemetry_0 = satellites.suchai_telemetry()
        self.satellites_print_timestamp_0 = satellites.print_timestamp('%Y-%m-%d %H:%M:%S')
        self.satellites_print_csp_message_0 = satellites.print_csp_message()
        self.satellites_fixedlen_tagger_0_0_0 = satellites.fixedlen_tagger('syncword', 'packet_len', packlen, numpy.byte)
        self.low_pass_filter_0 = filter.fir_filter_ccf(1, firdes.low_pass(
        	1, samp_rate, cutoff_freq, 1000, firdes.WIN_HAMMING, 6.76))
        self.digital_gmsk_demod_0 = digital.gmsk_demod(
        	samples_per_symbol=samples_symbol,
        	gain_mu=0.175,
        	mu=0.5,
        	omega_relative_limit=0.005,
        	freq_error=0.0,
        	verbose=False,
        	log=False,
        )
        self.digital_correlate_access_code_tag_bb_0_0_0 = digital.correlate_access_code_tag_bb(sync, threshold, 'syncword')
        self.blocks_wavfile_source_0 = blocks.wavfile_source('/home/carlos/Descargas/suchai_test.wav', False)
        self.blocks_unpacked_to_packed_xx_0 = blocks.unpacked_to_packed_bb(1, gr.GR_MSB_FIRST)
        self.blocks_throttle_0 = blocks.throttle(gr.sizeof_float*1, samp_rate,True)
        self.blocks_tagged_stream_to_pdu_0_0_0 = blocks.tagged_stream_to_pdu(blocks.byte_t, 'packet_len')
        self.blocks_tagged_stream_multiply_length_0 = blocks.tagged_stream_multiply_length(gr.sizeof_char*1, 'packet_len', 1/8.0)
        self.blocks_multiply_xx_0 = blocks.multiply_vcc(1)
        self.blocks_float_to_complex_0 = blocks.float_to_complex(1)
        self.blocks_conjugate_cc_0 = blocks.conjugate_cc()
        self.analog_sig_source_x_0 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, demod_freq, 1, 0)



        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.blocks_tagged_stream_to_pdu_0_0_0, 'pdus'), (self.satellites_u482c_decode_0, 'in'))
        self.msg_connect((self.satellites_print_timestamp_0, 'out'), (self.satellites_print_csp_message_0, 'in'))
        self.msg_connect((self.satellites_print_timestamp_0, 'out'), (self.satellites_suchai_telemetry_0, 'in'))
        self.msg_connect((self.satellites_suchai_telemetry_0, 'out'), (self.zeromq_pub_msg_sink_0, 'in'))
        self.msg_connect((self.satellites_u482c_decode_0, 'out'), (self.satellites_print_timestamp_0, 'in'))
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_multiply_xx_0, 0))
        self.connect((self.blocks_conjugate_cc_0, 0), (self.low_pass_filter_0, 0))
        self.connect((self.blocks_float_to_complex_0, 0), (self.blocks_multiply_xx_0, 1))
        self.connect((self.blocks_multiply_xx_0, 0), (self.blocks_conjugate_cc_0, 0))
        self.connect((self.blocks_tagged_stream_multiply_length_0, 0), (self.blocks_tagged_stream_to_pdu_0_0_0, 0))
        self.connect((self.blocks_throttle_0, 0), (self.blocks_float_to_complex_0, 0))
        self.connect((self.blocks_unpacked_to_packed_xx_0, 0), (self.blocks_tagged_stream_multiply_length_0, 0))
        self.connect((self.blocks_wavfile_source_0, 0), (self.blocks_throttle_0, 0))
        self.connect((self.digital_correlate_access_code_tag_bb_0_0_0, 0), (self.satellites_fixedlen_tagger_0_0_0, 0))
        self.connect((self.digital_gmsk_demod_0, 0), (self.digital_correlate_access_code_tag_bb_0_0_0, 0))
        self.connect((self.low_pass_filter_0, 0), (self.digital_gmsk_demod_0, 0))
        self.connect((self.satellites_fixedlen_tagger_0_0_0, 0), (self.blocks_unpacked_to_packed_xx_0, 0))

    def get_threshold(self):
        return self.threshold

    def set_threshold(self, threshold):
        self.threshold = threshold
        self.digital_correlate_access_code_tag_bb_0_0_0.set_threshold(self.threshold)

    def get_sync(self):
        return self.sync

    def set_sync(self, sync):
        self.sync = sync
        self.digital_correlate_access_code_tag_bb_0_0_0.set_access_code(self.sync)

    def get_samples_symbol(self):
        return self.samples_symbol

    def set_samples_symbol(self, samples_symbol):
        self.samples_symbol = samples_symbol

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate, self.cutoff_freq, 1000, firdes.WIN_HAMMING, 6.76))
        self.blocks_throttle_0.set_sample_rate(self.samp_rate)
        self.analog_sig_source_x_0.set_sampling_freq(self.samp_rate)

    def get_packlen(self):
        return self.packlen

    def set_packlen(self, packlen):
        self.packlen = packlen

    def get_demod_freq(self):
        return self.demod_freq

    def set_demod_freq(self, demod_freq):
        self.demod_freq = demod_freq
        self.analog_sig_source_x_0.set_frequency(self.demod_freq)

    def get_cutoff_freq(self):
        return self.cutoff_freq

    def set_cutoff_freq(self, cutoff_freq):
        self.cutoff_freq = cutoff_freq
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate, self.cutoff_freq, 1000, firdes.WIN_HAMMING, 6.76))


def main(top_block_cls=suchai, options=None):

    tb = top_block_cls()
    tb.start()
    tb.wait()


if __name__ == '__main__':
    main()
