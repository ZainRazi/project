# encoding: utf-8
import('bootstrap')
import('chromosome')
import('rng')

define_specification :representation do

  template_parameter :chromosome, s(:chromosome)

  # Generates a chromosome according to this representation.
  abstract_method :generate, returns: T[:chromosome], accepts: [
    parameter(:rng, s(:rng))
  ]

end

define_type :representation do

  attribute :chromosome, t(:chromosome)

  composer do
    s = spec(:representation).extend(name,
      chromosome: chromosome.compose
    )
  end

end
