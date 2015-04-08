# encoding: utf-8
import('../type/chromosome')

# A fixed-length vector of numerical values.
define_specification :vector do
  extends(:chromosome)

  # The type of value held within this array.
  template_parameter(:of, ruby(:*))

  # The contents of this array.
  attribute :contents, list(T[:of])

  # The minimum value that a component of this vector may assume.
  attribute :min_value, T[:of]

  # The maximum value that a component of this vector may assume.
  attribute :max_value, T[:of]

  # A flag indicating whether the range of values that this vector may assume
  # is inclusive.
  attribute :inclusive, ruby(:boolean)

  # Constructs a new vector chromosome.
  constructor [

    # The length of this vector.
    parameter(:length,    ruby(:integer)),

    # The minimum value that this vector may take.
    parameter(:min,       ruby(:integer), -2_147_483_648),

    # The maximum value that this vector may take.
    parameter(:max,       ruby(:integer), 2_147_483_647),

    # A flag indicating whether the range of legal vector
    # values should include its maximum value.
    parameter(:inclusive, ruby(:boolean), true)

  ] do
    self.min_value = min
    self.max_value = max
    self.inclusive = inclusive
    self.contents = Array.new(length)
  end

  # Returns a component of this vector at a specific index.
  method :[], returns: T[:of], accepts: [
    parameter(:index, ruby(:integer))
  ] do
    self.contents[index]
  end

  # Clamps a given component value within the legal range for component
  # values for this vector.
  method :clamp, returns: T[:of], accepts: [
    parameter(:value, T[:of])
  ] do
    if (inclusive && value > max_value) || (!inclusive && value >= max_value)
      return max_value
    elsif value < min_value
      return min_value
    else
      return value
    end
  end

  # Specifies the contents of this vector at a given index.
  method :[]=, accepts: [
    parameter(:index, ruby(:integer)),
    parameter(:value, T[:of])
  ] do
    self.contents[index] = clamp(value)
  end

  # Directly specifies the value for a component at a given index
  # within this vector, skipping any transformations, such as
  # clamping.
  method :set, accepts: [
    parameter(:index, ruby(:integer)),
    parameter(:value, T[:of])
  ] do
    self.contents[index] = value
  end

  # Returns the number of times that a given integer appears within
  # this vector.
  method :count, accepts: [
    parameter(:i, ruby(:integer))
  ] do
    contents.count(i)
  end

  # Returns the length of this vector.
  method :length, returns: ruby(:integer) do
    contents.length
  end

  method :splice, accepts: [
    parameter(:from, ruby(:integer)),
    parameter(:to, ruby(:integer)),
    parameter(:with, ruby(:*)),
    parameter(:inclusive, ruby(:boolean), false)
  ] do
    self.contents[inclusive ? (from .. to) : (from ... to)] = with.contents
  end

  # Returns a new vector containing each of the components within
  # a given range of indices from this vector.
  method :slice, returns: ruby(:*), accepts: [
    parameter(:from, ruby(:integer)),
    parameter(:to, ruby(:integer)),
    parameter(:inclusive, ruby(:boolean), false)
  ] do
    r = Chromosome::Vector[template(:of)].new(length, min_value, max_value, inclusive)
    r.contents = self.contents[inclusive ? (from .. to) : (from ... to)]
    return r
  end

  # Appends another vector to this one (non-destructively) and returns
  # the resulting vector.
  method :concat, returns: ruby(:*), accepts: [
    parameter(:with, ruby(:*))
  ] do
    r = Chromosome::Vector[template(:of)].new(length + with.length, min_value, max_value, inclusive)
    r.contents = self.contents + with.contents
    return r
  end

  # Produces a deep clone of this vector.
  method :clone do
    c = Chromosome::Vector[template(:of)].new(length, min_value, max_value, inclusive)
    c.contents = self.contents.clone
    return c
  end

  # Returns a string-based description of this vector.
  method :to_s, returns: ruby(:string) do
    contents.to_s
  end

end

# Registers vector as a type of chromosome.
define_type :vector do
  extends(:chromosome)

  # The type of value that the vector holds.
  attribute :of, ruby(:*)
  template lambda { |base, of| base.of(of) }

end
