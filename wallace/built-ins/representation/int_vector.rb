# encoding: utf-8
import('../type/representation')
import('../chromosome/vector')

define_type :int_vector do
  extends(:representation)

  composer do
    s(:representation, :int_vector)
  end

end

define_specification :int_vector do
  extends(:representation,
    chromosome: s(:chromosome, :vector).instantiate(of: ruby(:int)))

  # The length of the vector.
  attribute :length, ruby(:integer)

  # The minimum value that a component within this vector may assume.
  attribute :min, ruby(:integer)

  # The maximum value that a component within this vector may assume.
  attribute :max, ruby(:integer)

  # A flag indicating whether the range of values that this vector may assume
  # is inclusive.
  attribute :inclusive, ruby(:boolean)

  # Constructs a new int vector representation.
  constructor [
    parameter(:length,    ruby(:integer)),
    parameter(:min,       ruby(:integer), 0),
    parameter(:max,       ruby(:integer), 2_147_483_647),
    parameter(:inclusive, ruby(:boolean), true)
  ] do
    self.min = min
    self.max = max
    self.inclusive = inclusive
    self.length = length
  end

  # Generates a new int vector at pseudo-random.
  implement_method :generate do
    c = Chromosome::Vector[Integer].new(length, min, max, inclusive)
    for i in 0 ... length
      c.set(i, rng.integer(min, max, inclusive))
    end
    return c
  end

  method :sample_gene, returns: ruby(:integer), accepts: [
    parameter(:rng, s(:rng))
  ] do
    rng.integer(min, max, inclusive)
  end

end
