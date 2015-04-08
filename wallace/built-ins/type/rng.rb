# encoding: utf-8
import('bootstrap')

define_specification :rng do

  abstract_method :integer, returns: ruby(:integer), accepts: [
    parameter(:minimum, ruby(:integer), -2_147_483_648),
    parameter(:maximum, ruby(:integer), 2_147_483_647),
    parameter(:inclusive, ruby(:boolean), true)
  ]

  abstract_method :float, returns: ruby(:float), accepts: [
    parameter(:minimum, ruby(:float), 0.0),
    parameter(:maximum, ruby(:float), 1.0),
    parameter(:inclusive, ruby(:boolean), true)
  ]

  abstract_method :sample, returns: ruby(:*), accepts: [
    parameter(:from, list(ruby(:*))),
    parameter(:number, ruby(:integer), 1)
  ]

  abstract_method :gaussian, returns: ruby(:float), accepts: [
    parameter(:mean,  ruby(:float)),
    parameter(:std,   ruby(:float))
  ]

end

define_type :rng do
  composer { s(:rng) }
end
