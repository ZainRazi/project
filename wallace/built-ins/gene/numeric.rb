# encoding: utf-8
# import('../type/gene')

# define_type :numeric do
#   extends(:gene)
# end

# define_specification :numeric do
#   extends(:gene)

#   # The minimum value that a gene may assume.
#   attribute :min, T[:t]

#   # The maximum value that a gene may assume.
#   attribute :max, T[:t]

#   constructor [
#     parameter(:min,       T[:t]),
#     parameter(:max,       T[:t]),
#     parameter(:inclusive, ruby(:boolean), true)
#   ] do
#     self.min = min
#     self.max = max
#     self.inclusive = inclusive
#   end

#   # Samples a new gene value at random.
#   abstract_method :sample, returns: T[:t], accepts: [
#     parameter(:rng, s(:rng))
#   ]

# end
