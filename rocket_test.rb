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
      :saturation, :int,
      :c, [:int32, 100],
      :taps, [:int32, 100]

    def init(ntaps)
      Rocket.init_fir_filter(self, ntaps)
    end

    def run(s)
      Rocket.run_fir_filter(self, s)
    end
  end

  class BaroData < FFI::Struct
    include AttrAccessorHelper

    layout :alt, :int32,
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

# Sinusoidal noise generator
def noise(tick, amp_mm, period)
  return (amp_mm * Math.sin(2.0 * Math::PI * (tick.to_f / period.to_f))).to_i
end

def add_noise(tick, true_alt)
  alt = true_alt
  alt += noise(tick, 400, 5030)
  alt += noise(tick, 800, 12175)
  alt += noise(tick, 1200, 18050)
  alt += noise(tick, 1085, 10000)
  alt += 200 * (Random.random_number - 0.5)
  return alt.to_i
end

def generate_test_signal
  true_alt = 250000

  # ~20 minutes at ground level, unmoving
  signal = []
  tick = 0
  12000.times do 
    tick += 90 + Random.rand(30)
    alt = add_noise(tick, true_alt)
    signal << [tick, alt]
  end

  # Ascend 5m over 10 seconds
  100.times do 
    tick += 90 + Random.rand(30)
    true_alt += 50
    alt = add_noise(tick, true_alt)
    signal << [tick, alt]
  end

  6000.times do
    tick += 90 + Random.rand(30)
    alt = add_noise(tick, true_alt)
    signal << [tick, alt]
  end

  # Jump up 2m in 1 second
  10.times do
    tick += 90 + Random.rand(3)
    true_alt += 50
    alt = add_noise(tick, true_alt)
    signal << [tick, alt]
  end

  600.times do
    tick += 90 + Random.rand(3)
    alt = add_noise(tick, true_alt)
    signal << [tick, alt]
  end

  signal
end

def interpolate interp, s
  samples_i = []
  s.each do |t, s|
    interp.feed(s, t)
    while not (r = interp.get).nil?
      samples_i << [r.last, r.first]
    end
  end
  samples_i
end

def dump_to_file samples, name
  File.open(name, "w") do |f|
    samples.each do |t, s|
      f.puts "#{t}, #{s}"
    end
  end
  nil
end

def setup
  samples = generate_test_signal[0..1000]
  interp = Rocket::Interpolator.new
  interp.init(25)
  return samples, interp
end

def baro_iteration state, sample, tick
  state.update_baro(sample, tick)
  return state.baro.tick, state.baro.alt, state.baro.vspeed
end

def generate_acceleration alt, tick, n=25, accel=4000, vs=0, jitter=200
  s = []
  n.times do
    tick += Random.rand(30) + 90
    vs += accel + (Random.random_number * jitter).to_i
    if vs < -2000
      vs = -2000 + (Random.random_number * 100).to_i
    end
    alt += vs / 10
    s << [tick, alt]
    if alt < 250000
      break
    end
  end
  [s, vs]
end

def make_test period, ntaps
  samples, interp = setup
  interp.init period
  filter = Rocket::FirFilter.new
  filter.init ntaps

  samples_i = interpolate interp, samples
  samples_f = samples_i.map{|t, s| [t, filter.run(s)]}

  dump_to_file samples, "samples.dat"
  dump_to_file samples_i, "samples_i.dat"
  dump_to_file samples_f, "samples_f.dat"

  return samples, samples_i, samples_f
end

def make_test2
  samples = generate_test_signal
  accel, vs = generate_acceleration samples.last.last, samples.last.first
  accel.map!{|t, s| [t, add_noise(t, s)]}
  samples += accel
  coast, vs = generate_acceleration accel.last.last, accel.last.first, 12000, -1000, vs
  coast.map!{|t, s| [t, add_noise(t, s)]}
  samples += coast
  samples
end

samples = make_test2
rs = Rocket::RocketState.new
rs.init
rs.baro.field_alt_interp.init 2000

i = 0
while i < samples.length do
  if i % (150 + Random.rand(10)) == 0
    (10 + Random.rand(3)).times do
      samples.delete_at(i)
    end
  end
  i += 1
end

states = []
samples.each do |t, s|
  old_phase = rs.phase
  phase = rs.update_baro(s, t)
  rs.phase = :armed if states.length == 15000
  rs.phase = phase if phase != old_phase
  if rs.phase != old_phase
    puts "Phase transition occurred!"
    puts rs
  end
  puts rs if states.length < 3
  states << rs.dup
end

last = samples.first
naive_speed =  samples[1..].map do |t, s|
  dt = t - last.first
  ds = s - last.last
  last = [t, s]
  [t, (ds * 1000)/dt]
end

dump_to_file samples, 'samples.dat'
dump_to_file naive_speed, 'vspeed_unfiltered.dat'

dump_to_file states.map{|s| [s.baro.tick, s.baro.alt_raw]}, 'alt_raw.dat'
dump_to_file states.map{|s| [s.baro.tick, s.baro.alt]}, 'alt.dat'
dump_to_file states.map{|s| [s.baro.tick, s.baro.vspeed]}, 'vspeed.dat'
dump_to_file states.map{|s| [s.baro.tick, s.baro.alt_field]}, 'field_alt.dat'
dump_to_file states.map{|s| [s.baro.tick, 20000 * Rocket::Phase[s.phase]]}, 'phase.dat'
