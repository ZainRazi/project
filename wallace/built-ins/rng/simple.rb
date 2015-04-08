# encoding: utf-8
import('../type/rng')

define_specification :simple do
  extends(:rng)

  attribute :generator, ruby(:random)

  # Add seeding!
  constructor [] do
    self.generator = Random.new
  end
  
  implement_method :float do
    generator.rand(inclusive ? (minimum .. maximum) : (minimum ... maximum))
  end

  implement_method :integer do
    i = generator.rand(inclusive ? (minimum .. maximum) : (minimum ... maximum))
  end

  implement_method :sample do
    from.sample(number, random: generator)
  end

  implement_method :gaussian do
    theta = 2 * Math::PI * generator.rand
    rho = Math.sqrt(-2 * Math.log(1 - generator.rand))
    scale = std * rho
    return mean + scale * Math.sin(theta)
  end

end

define_type :simple do
  extends(:rng)
  composer { s(:rng, :simple) }
end
