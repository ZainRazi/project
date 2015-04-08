# encoding: utf-8
# import('numeric')

# define_type :float do
#   extends(:gene, :numeric)
#   ruby_type(ruby(:float))
#   composer { s(:gene, :float) }
# end

# define_specification :float do
#   extends(:gene, :numeric, [t: ruby(:float)])
#   implement_method :sample do
#     rng.float(min, max, inclusive)
#   end
# end
