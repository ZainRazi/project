# encoding: utf-8
import('bootstrap')
import('rng')
import('statistics')
import('population')

define_specification :criterion do
  abstract_method :satisfied?, returns: ruby(:boolean), accepts: [
    p(:rng,         s(:rng)),
    p(:statistics,  s(:statistics)),
    p(:population,  s(:population))
  ]
end

define_type :criterion do
  attribute :satisfied?, ruby(:block)
  composer do
    s = spec(:criterion).extend(name)
    s.implement_method(:satisfied?, &satisfied?)
    s
  end
end
