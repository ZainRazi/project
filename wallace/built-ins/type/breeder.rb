# encoding: utf-8
import('bootstrap')
import('rng')
import('statistics')
import('deme')
import('individual')

define_specification :breeder do

  #template_parameter :individual, s(:individual)

  # Produces and returns offspring for a given deme.
  abstract_method :breed, returns: T[:individual], accepts: [
    parameter(:rng,         s(:rng)),
    parameter(:statistics,  s(:statistics)),
    parameter(:deme,        s(:deme))
  ]

end

define_type :breeder do

  composer do
    s(:breeder)
  end

end
