require 'ffi'


module AttrAccessorHelper
  # This method can also take a &block, but we would not use it.
  def method_missing( sym, *args )
    # convert symbol to a string to allow regex checks
    str = sym.to_s

    # derive the member's symbolic name
    member = str.match( /^([a-z0-9_]+)/i )[1].to_sym

    # raise an exception via the default behavior if that symbol isn't a member!
    super unless members.include? member

    # this ternary checks for the presence of an equals sign (=) to indicate an
    # assignment operation and changes which method we invoke and whether or not
    # to send the splatted arguments as well.
    (str =~ /=$/) ? send( :[]=, member, *args ) : send( :[], member )
  end
end

module Rocket
  extend FFI::Library
  ffi_lib "./librocket.so"
  Phase = enum [ 
    :idle,
    :armed,
    :ascent,
    :descent,
    :landed
  ]

  class Interpolator < FFI::Struct
    include AttrAccessorHelper

    layout :period, :uint32,
      :next_tick, :uint64,
      :real_last, :int32,
      :real_last_tick, :uint64,
      :current, :int32,
      :current_tick, :uint64,
      :valid, :int

    def to_s
      <<~EOF
      Period #{period} (#{valid != 0 ? 'Valid' : 'Invalid'})
      Last data (tick): \t#{real_last} \t(#{real_last_tick})
      Current data (tick): \t#{current} \t(#{current_tick})
      Next tick to generate: \t#{next_tick}
      EOF
    end

    def feed(sample, tick)
      Rocket.feed_interp(self, sample, tick)
    end

    def get
      sample = FFI::MemoryPointer.new(:int32)
      tick = FFI::MemoryPointer.new(:uint64)
      res = Rocket.get_interp(self, sample, tick)
      return nil if res == 0
      return sample.get_int32(0), tick.get_uint64(0)
    end

    def init(pd)
      Rocket.init_interp(self)
      self.period = pd
    end
  end

  class FirFilter < FFI::Struct
    include AttrAccessorHelper
    layout :ntaps, :int,
      :valid, :int,
      :c, [:int32, 100],
      :taps, [:int32, 100]

    def init(ntaps)
      Rocket.init_fir_filter(self, ntaps)
    end

    def run(s)
      Rocket.run_fir_filter(self, s)
    end

    def set_coefficients(coffs)
      self.ntaps=coffs.size
      self.c.to_ptr.write_array_of_int32(coffs)
    end
  end

  class BaroData < FFI::Struct
    include AttrAccessorHelper

    layout :alt, :int32,
      :alt2, :int32,
      :alt_max, :int32,
      :alt_raw, :int32,
      :alt_raw_max, :int32,
      :alt_field, :int32,
      :tick, :uint64,
      :vspeed, :int32,
      :vspeed_max, :int32,
      :landing_tick, :uint64,
      :alt_interp, Interpolator,
      :alt_filter, FirFilter,
      :field_alt_interp, Interpolator,
      :field_alt_filter, FirFilter,
      :apogee_detect_filter, FirFilter,
      :valid, :int

    def to_s
      s = <<~EOF
        Tick: #{tick}
        Alt (max): #{alt} (#{alt_max})
        VSpeed (max): #{vspeed} (#{vspeed_max})
        Field alt: #{alt_field}
        Landing tick: #{landing_tick}
        #{valid != 0 ? 'Valid' : 'Invalid'}
      EOF
    end
  end


  class GpsData < FFI::Struct
    include AttrAccessorHelper

    layout :lat, :int32,
      :lon, :int32,
      :alt, :int32,
      :alt_max, :int32,
      :time, :int64,
      :tick, :uint64,
      :valid, :int
  end

  class RocketState < FFI::Struct
    include AttrAccessorHelper

    layout :phase, Phase,
      :gps, GpsData,
      :baro, BaroData

    def to_s
      s = <<~EOF
        ObjectId: #{object_id}
        Phase: #{phase}
        GPS: #{gps}
        Baro: 
        #{baro.to_s.lines.map{|l| "\t" +l}.join}
      EOF
    end

    def init
      Rocket.init_state(self)
    end

    def update_baro alt, tick
      Rocket.update_baro self, alt, tick
    end
  end

  attach_function :init_state, [RocketState.by_ref], :void
  attach_function :update_baro, [RocketState.by_ref, :int32, :uint64], Phase

  attach_function :init_interp, [Interpolator.by_ref], :void
  attach_function :feed_interp, [Interpolator.by_ref, :int32, :uint64], :void
  attach_function :get_interp, [Interpolator.by_ref, :pointer, :pointer], :int

  attach_function :init_fir_filter, [FirFilter.by_ref, :int], :void
  attach_function :run_fir_filter, [FirFilter.by_ref, :int32], :int32
end
