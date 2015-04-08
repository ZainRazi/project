# encoding: utf-8
import('bootstrap')

define_type :objective do

  attribute :descriptor, ruby(:symbol)
  attribute :maximise, ruby(:boolean)
  attribute :function, ruby(:block)

  # Composes this objective into its own objective class.
  composer do
    s = spec(:objective).extend(name)
    s.implement_method(:compute, &function)
    s.constructor([], source: "self.maximise = #{maximise.inspect}")
    s
  end

  template lambda { |base, opts = {}, &fun|
    opts[:maximise] = true unless opts.key?(:maximise)
    base.maximise(opts[:maximise])
    base.function(fun)
  }

end

define_specification :objective do

  attribute :name,      ruby(:symbol)
  attribute :maximise,  ruby(:boolean)

  abstract_method :compute, returns: ruby(:float), accepts: [
    p(:rng,             s(:rng)),
    p(:population,      s(:population)),
    p(:statistics,      s(:statistics)),
    p(:representation,  s(:representation)),
    p(:chromosome,      s(:chromosome)) # need more info!  
  ]

end
