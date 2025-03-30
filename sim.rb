#! /usr/bin/env ruby

require 'csv'

require './rocket.rb'

def normalize_samples samples
  average = samples.map(&:last).sum / samples.length
  start = samples.first.first
  samples.map{|t, s| [t - start, s - average]}
end

def dump_samples samples, filename
  IO.write(filename, samples.map{|t, s| "#{t}\t#{s}"}.join("\n"))
end

def load_samples filename
  IO.readlines(filename).map(&:split).map{|s| s.map(&:to_i)}
end

def get_interpolated samples, tick, infinite: false
  if tick > samples.last.first and not infinite
    raise RangeError.new
  end

  # Loop sample set
  while tick > samples.last.first
    tick -= samples.last.first
  end

  last = samples.first
  samples.each do |t, s|
    return s if t == tick
    if last.first < tick && t > tick
      delta = s - last.last
      tdelta = t - last.first
      diff = tick - last.first
      return ((delta * diff) / tdelta) + last.last
    end
    last = [t, s]
  end
  p samples
  p tick
  p infinite
  raise RangeError.new
end

class SampleSet
  attr_reader :duration
  attr_reader :alt_begin
  attr_reader :alt_end
  attr_reader :samples
  def initialize filename, infinite: false, offset: 0
    @samples = load_samples filename
    @infinite = infinite
    @filename = filename
    @cache = {}
    @duration = @samples.last.first
    @alt_begin = @samples.first.last + offset
    @alt_end = @samples.last.last + offset
    @offset = offset
  end

  def [] i
    cached = @cache[i % @duration]
    return cached unless cached.nil?
    res = get_interpolated @samples, i, infinite: @infinite
    res += @offset
    @cache[i] = res
    res
  end

  def inspect
    "#<#{self.class}:#{object_id} #{@filename} " \
      "#{@samples.length} samples infinite:#{@infinite}>"
  end
end

class NoiseModel < SampleSet
  def initialize baseline=:idle
    super "noise-data/#{baseline.to_s}.dat", infinite: true
    @filename = ":#{baseline}"
  end
end


class AltitudeModel
  attr_reader :duration
  attr_reader :periods
  def initialize alt=250000
    @alt = alt
    @periods = []
    @duration = 0
  end

  def generate interval: 100, jitter: 10
    t = 0
    samples = []
    while t < duration
      samples << [t, self[t]]
      t += interval - (jitter/2) + Random.rand(jitter)
    end
    samples
  end

  def dump filename, interval: 100, jitter: 10
    dump_samples generate(interval: interval, jitter: jitter), filename
  end

  def level duration, noise: NoiseModel.new
    @periods << [
      @duration,
      :flat,
      @alt,
      noise
    ]
    @duration += duration
    self
  end

  def rate d, rate, noise: NoiseModel.new
    @periods << [
      @duration,
      :change,
      @alt,
      noise,
      rate
    ]
    @duration += d
    @alt += (d* rate).to_i
    self
  end

  def profile filename, noise: NoiseModel.new
    s = SampleSet.new filename
    @periods << [
      @duration,
      :samples,
      @alt,
      noise,
      s
    ]
    @duration += s.duration
    @alt += s.alt_end
    self
  end

  def [] i
    period = @periods.first
    @periods[1..].each do |p|
      break if p.first >= i
      period = p
    end

    alt = period[2]
    if period[1] == :change
      rate = period[4]
      delta = (rate * (i - period[0])).to_i
      alt += delta
    end

    if period[1] == :samples
      alt += period[4][i - period[0]]
    end

    alt += period[3][i] unless period[3].nil?
    alt
  end
end

def csv2dat filename
  csv = CSV.new(File.open(filename), skip_lines: /^#/)
  dat = csv.read.map{|l| l.map{|x| (x.to_f * 1000).to_i}}
  start = dat.first.first
  dat = dat.map{|t, s| [t - start, s].join("\t")}
  csv.close
  IO.write(filename.gsub(/csv$/, 'dat'), dat.join("\n"))
end

def standard_field_model
  m = AltitudeModel.new
  noise_idle = NoiseModel.new
  noise_heavy = NoiseModel.new :very_disturbed
  m.level 600_000, noise: noise_idle
  m.rate 10_000, 0.2, noise: noise_idle
  m.level 30_000, noise: noise_idle
  m.rate 30_000, -0.01, noise: noise_heavy
  m.rate 15_000, 0.08, noise: noise_heavy
  m.level 15_000, noise: noise_idle
  m
end

def run_rocket_with_model rocket, model, basename, 
    interval: 100, jitter: 20, arm_at: 60_000
  basename = "sim-output/" + basename
  file_i = File.open(basename + '-input.dat', "w")
  file_alt = File.open(basename + '-alt.dat', "w")
  file_vspeed = File.open(basename + '-vspeed.dat', "w")
  file_vspeed_i = File.open(basename + '-vspeed_unfiltered.dat', "w")

  t = 0
  t_last = 0
  10.times do
    t_last = t
    t += interval - (jitter/2) + Random.rand(jitter)
  end

  armed = false
  puts rocket
  steps = 0
  s_last = model[t]
  while t < model.duration
    steps += 1
    if t > arm_at and not armed
      rocket.phase = :armed
      armed = true
      puts rocket
    end

    s = model[t]
    phase = rocket.update_baro(s, t)
    if phase != rocket.phase
      puts "Phase change"
      rocket.phase = phase
      puts rocket
    end

    file_i.puts("#{t}\t#{s}")
    file_alt.puts("#{t}\t#{rocket.baro.alt}")
    file_vspeed.puts("#{t}\t#{rocket.baro.vspeed}")
    file_vspeed_i.puts("#{t}\t#{((s_last - s) * 1000)/(t_last - t)}")

    t_last = t
    s_last = s
    t += interval - (jitter/2) + Random.rand(jitter)
  end

  puts "Ran #{steps} steps to a time of #{t}"

  file_i.close
  file_alt.close
  file_vspeed.close
  file_vspeed_i.close
end

if __FILE__ == $0
  model = standard_field_model
  model.profile('flight-profile.dat', noise: NoiseModel.new(:very_disturbed))
  model.level(60_000)

  rocket = Rocket::RocketState.new
  rocket.init
  run_rocket_with_model rocket, model, 'default'

  gnuplot_command = "plot 'sim-output/default-input.dat', "\
    "'sim-output/default-alt.dat', 'sim-output/default-vspeed.dat', "\
    "'sim-output/default-vspeed_unfiltered.dat';"

  `gnuplot -e "#{gnuplot_command}; pause -1"`
end
