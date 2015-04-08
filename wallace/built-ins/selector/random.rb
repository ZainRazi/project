# encoding: utf-8
import('../type/selector')

define_specification :random do
  extends(:selector)

  implement_method :prepare do
    return candidates
  end

  implement_method :select do
    return rng.sample(candidates, num_requested)
  end
end

define_type :random do
  extends(:selector)
  composer { s(:selector, :random) }
end
